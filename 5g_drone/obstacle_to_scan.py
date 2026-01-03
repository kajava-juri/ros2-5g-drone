#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import Mavlink
from sensor_msgs.msg import LaserScan
import struct
import math


class ObstacleToScan(Node):
    def __init__(self):
        super().__init__('obstacle_to_scan')
        
        # Match the publisher's QoS settings
        mavlink_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.mavlink_sub = self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_callback,
            mavlink_qos)
        
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        self.get_logger().info('OBSTACLE_DISTANCE to LaserScan converter started')
        self._msg_count = 0
    
    def mavlink_callback(self, msg):
        # OBSTACLE_DISTANCE message ID is 330
        if msg.msgid == 330:
            self.parse_and_publish(msg)
    
    def parse_and_publish(self, msg):
        try:
            # The Mavlink message has sysid, compid, msgid, and payload64
            # payload64 contains the actual message data packed as uint64 array
            
            # Convert payload64 array to bytes
            data = bytearray()
            for val in msg.payload64:
                data.extend(struct.pack('<Q', val))
            
            # Trim to actual message length (OBSTACLE_DISTANCE is 158 bytes)
            data = bytes(data[:158])
            
            # MAVLink OBSTACLE_DISTANCE message format (common.xml):
            # Field ordering (all little-endian):
            # uint64_t time_usec           [0:8]
            # uint16_t distances[72]       [8:152]
            # uint16_t min_distance        [152:154]
            # uint16_t max_distance        [154:156]
            # uint8_t sensor_type          [156:157]
            # uint8_t increment            [157:158]
            
            time_usec = struct.unpack('<Q', data[0:8])[0]
            distances_raw = struct.unpack('<72H', data[8:152])
            min_distance = struct.unpack('<H', data[152:154])[0]
            max_distance = struct.unpack('<H', data[154:156])[0]
            sensor_type = struct.unpack('<B', data[156:157])[0]
            increment = struct.unpack('<B', data[157:158])[0]
            
            self._msg_count += 1
            
            if self._msg_count % 20 == 0:
                self.get_logger().info(
                    f'Parsed: min_dist={min_distance}cm, max_dist={max_distance}cm, '
                    f'increment={increment}deg, sensor_type={sensor_type}'
                )
            
            # Create LaserScan
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = 'base_link'
            
            # Angle setup - use increment if valid, otherwise default to 5 degrees
            increment_deg = increment if increment > 0 else 5.0
            increment_rad = math.radians(increment_deg)
            
            scan.angle_min = 0.0
            scan.angle_max = 2 * math.pi
            scan.angle_increment = increment_rad
            
            # Range limits (cm to meters)
            scan.range_min = min_distance / 100.0 if min_distance > 0 else 0.1
            scan.range_max = max_distance / 100.0 if max_distance > 0 else 30.0
            
            # Sanity check
            if scan.range_max <= scan.range_min:
                scan.range_min = 0.1
                scan.range_max = 30.0
            
            # Convert distances: cm to meters, 65535 (UINT16_MAX) = no obstacle
            scan.ranges = [
                d / 100.0 if d < 65535 else float('inf')
                for d in distances_raw
            ]
            
            self.scan_pub.publish(scan)
            
            if self._msg_count % 20 == 0:
                valid_ranges = sum(1 for r in scan.ranges if r != float('inf'))
                avg_range = sum(r for r in scan.ranges if r != float('inf')) / max(valid_ranges, 1)
                self.get_logger().info(
                    f'Published: {valid_ranges}/72 valid, avg={avg_range:.2f}m, '
                    f'range: {scan.range_min:.2f}-{scan.range_max:.2f}m'
                )
                
        except Exception as e:
            self.get_logger().error(f'Error parsing OBSTACLE_DISTANCE: {e}', exc_info=True)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleToScan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()