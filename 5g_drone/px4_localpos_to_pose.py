import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from px4_msgs.msg import VehicleLocalPosition
from tf_transformations import quaternion_from_euler


class LocalPosToPose(Node):
    def __init__(self):
        super().__init__("px4_localpos_to_pose")

        self.frame_id = self.declare_parameter("frame_id", "map").value

        # Use BEST_EFFORT + VOLATILE for compatibility with rosbag playback
        # TRANSIENT_LOCAL has timing issues with rosbag2_player
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.cb,
            qos_profile
        )
        
        self.message_count = 0
        
        # Timer to verify node is alive
        self.create_timer(2.0, self.timer_callback)
        
        self.get_logger().info("Subscribed to /fmu/out/vehicle_local_position with QoS:")
        self.get_logger().info(f"  Reliability: {qos_profile.reliability}")
        self.get_logger().info(f"  Durability: {qos_profile.durability}")

        self.pub_pose = self.create_publisher(PoseStamped, "/drone/local_pose", 10)
        self.pub_path = self.create_publisher(Path, "/drone/path", 10)
        
        self.get_logger().info("Node initialized, waiting for messages...")

        self.path = Path()
        self.path.header.frame_id = self.frame_id

    def timer_callback(self):
        self.get_logger().info(f"Node alive. Messages received so far: {self.message_count}")

    def cb(self, msg: VehicleLocalPosition):
        self.message_count += 1
        self.get_logger().info(f"Callback triggered! Received position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        # PX4 local position is in NED.
        # We'll publish it as-is into RViz in "map" frame.
        # (Later you can convert NED->ENU if you want.)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(msg.z)

        # Heading/yaw: msg.heading is typically radians
        yaw = float(msg.heading) if msg.heading == msg.heading else 0.0  # nan check
        q = quaternion_from_euler(0.0, 0.0, yaw)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pub_pose.publish(pose)
        print(f"Published local pose: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}, yaw={yaw:.2f}")

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)

        # limit path length
        if len(self.path.poses) > 5000:
            self.path.poses = self.path.poses[-5000:]

        self.pub_path.publish(self.path)


def main():
    rclpy.init()
    node = LocalPosToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
