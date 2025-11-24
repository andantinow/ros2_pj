#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive_safety',
            10)
        
        self.ttc_threshold = 0.6 

    def scan_callback(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        
        ranges[ranges == 0] = 100.0 
        ranges[np.isnan(ranges)] = 100.0
        
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)
        

    def emergency_brake(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        self.drive_pub.publish(msg)
        self.get_logger().warn('EMERGENCY BRAKE ACTIVATED')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
