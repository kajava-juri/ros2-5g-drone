#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class JoystickOffb(Node):
    def __init__(self):
        super().__init__('joystick_offb')
        
        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        state_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # State
        self.current_state = State()
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, state_qos_profile)
        self.last_mode_request_time = self.get_clock().now()
        self.last_arm_request_time = self.get_clock().now()
        self.mode_request_interval = 5.0  # seconds
        self.arm_request_interval = 5.0
        self.mode_sent = False
        self.armed_sent = False
        self.pose_count = 0
        self.offboard_initiated = False
        self.arming_initiated = False
        
        # Button state tracking for debouncing
        self.last_button_states = []
        
        # Timer for sending setpoints (must be >2Hz for offboard)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        # Current velocity command
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        
        self.max_velocity = 3.0
        self.max_yaw_rate = 0.5  # rad/s
        
        self.get_logger().info('Joystick offboard control started')

    def timer_callback(self):
        # Continuously send setpoints (required for offboard mode)
        self.send_setpoint()
        
        # Handle offboard sequence after sufficient setpoints
        if self.offboard_initiated:
            if self.pose_count < 100:
                self.pose_count += 1
                if self.pose_count == 100:
                    self.get_logger().info('100 setpoints sent, ready to set offboard mode')
            elif self.pose_count >= 100:
                if self.current_state.mode != "OFFBOARD" and not self.mode_sent:
                    self.set_offboard_mode()
                elif self.current_state.mode == "OFFBOARD":
                    # Offboard mode set, reset flags
                    self.offboard_initiated = False
                    self.mode_sent = False
                    self.get_logger().info('Offboard mode sequence complete')
        
        # Handle arming sequence (only if already in offboard mode)
        if self.arming_initiated:
            if self.current_state.mode == "OFFBOARD":
                if not self.current_state.armed and not self.armed_sent:
                    self.arm_drone()
                elif self.current_state.armed:
                    # Arming complete, reset flags
                    self.arming_initiated = False
                    self.armed_sent = False
                    self.get_logger().info('Arming sequence complete')
            else:
                self.get_logger().warn('Cannot arm: not in offboard mode')
                self.arming_initiated = False
    
    def state_cb(self, msg):
        self.current_state = msg
    
    def joy_callback(self, msg):
        deadzone = 0.15
        
        def apply_deadzone(val):
            return val if abs(val) > deadzone else 0.0
        
        # Read joystick 
        # TODO: map the axes/buttons
        forward = apply_deadzone(-msg.axes[1])
        right = apply_deadzone(msg.axes[0])
        up = apply_deadzone(-msg.axes[3])
        yaw = apply_deadzone(msg.axes[2])
        
        # Scale to velocities
        self.vx = forward * self.max_velocity
        self.vy = right * self.max_velocity
        self.vz = -up * self.max_velocity  # NED frame
        self.vyaw = yaw * self.max_yaw_rate
        
        # Initialize last_button_states if needed
        if len(self.last_button_states) != len(msg.buttons):
            self.last_button_states = [0] * len(msg.buttons)
        
        # Log all button states
        for i in range(len(msg.buttons)):
            if msg.buttons[i] == 1:
                self.get_logger().info(f'Button {i} pressed')
        
        # Button handling with debouncing (trigger only on button press, not hold)
        if len(msg.buttons) > 9:
            # Button 8: Set offboard mode
            if msg.buttons[5] == 1 and self.last_button_states[8] == 0:
                if not self.offboard_initiated and self.current_state.mode != "OFFBOARD":
                    self.get_logger().info('Starting offboard mode sequence...')
                    self.offboard_initiated = True
                    self.pose_count = 0
            
            # Button 9: Arm drone
            if msg.buttons[9] == 1 and self.last_button_states[9] == 0:
                if not self.arming_initiated and not self.current_state.armed:
                    self.get_logger().info('Starting arming sequence...')
                    self.arming_initiated = True
            
            # Button 7: Land
            if msg.buttons[7] == 1 and self.last_button_states[7] == 0:
                self.land_drone()
        
        # Update last button states
        self.last_button_states = list(msg.buttons)
    
    def send_setpoint(self):
        """Send velocity setpoint at 20Hz"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Body frame
        
        msg.twist.linear.x = self.vx
        msg.twist.linear.y = self.vy
        msg.twist.linear.z = self.vz
        msg.twist.angular.z = self.vyaw
        
        self.vel_pub.publish(msg)
    
    def mode_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.mode_sent = True
                self.get_logger().info("mode enabled successfully")
            else:
                self.get_logger().warn("mode request rejected")
                self.mode_sent = False  # Reset flag on rejection to retry
        except Exception as e:
            self.get_logger().error(f"mode service call failed: {e}")
            self.mode_sent = False  # Reset flag on error to retry

    def arm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.armed_sent = True
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().warn("Vehicle arming request rejected")
                self.armed_sent = False  # Reset flag on rejection to retry
        except Exception as e:
            self.get_logger().error(f"Vehicle arming service call failed: {e}")
            self.armed_sent = False  # Reset flag on error to retry

    def disarm_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle disarmed successfully - Mission complete!")
            else:
                self.get_logger().warn("Vehicle disarming request rejected")
        except Exception as e:
            self.get_logger().error(f"Vehicle disarming service call failed: {e}")
    
    def set_offboard_mode(self):
        """Set drone to offboard mode"""
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.mode_callback)
    
    def arm_drone(self):
        """Arm the drone"""
        req = CommandBool.Request()
        req.value = True
        
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_callback)
    
    def land_drone(self):
        """Land the drone"""
        req = SetMode.Request()
        req.custom_mode = 'AUTO.LAND'
        self.set_mode_client.call_async(req)
        self.get_logger().info('Landing...')


def main(args=None):
    rclpy.init(args=args)
    node = JoystickOffb()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()