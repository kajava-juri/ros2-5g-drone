#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition


class TestSubscriber(Node):
    def __init__(self):
        super().__init__('test_subscriber')
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.count = 0
        
        self.sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.callback,
            qos
        )
        
        self.get_logger().info('Test subscriber initialized')
        self.create_timer(1.0, self.timer_cb)
    
    def timer_cb(self):
        self.get_logger().info(f'Messages received: {self.count}')
    
    def callback(self, msg):
        self.count += 1
        self.get_logger().info(f'GOT MESSAGE #{self.count}: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')


def main():
    rclpy.init()
    node = TestSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
