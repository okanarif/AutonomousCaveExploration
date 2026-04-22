#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3
import sys

class TakeoffPublisher(Node):
    def __init__(self):
        super().__init__('takeoff_publisher')
        
        # Subscribe to current state
        self.sub = self.create_subscription(
            Odometry, 'current_state_est',
            self.state_callback, 10)
        
        # Publisher for desired state
        self.pub = self.create_publisher(
            MultiDOFJointTrajectoryPoint,
            'desired_state', 10)
        
        self.got_state = False
        
    def state_callback(self, msg):
        if not self.got_state:
            self.got_state = True
            
            # TAKEOFF COMMAND: Set desired position 2m above current
            desired = MultiDOFJointTrajectoryPoint()
            
            transform = Transform()
            transform.translation.x = msg.pose.pose.position.x
            transform.translation.y = msg.pose.pose.position.y
            transform.translation.z = msg.pose.pose.position.z + 2.0 #+ 2.0  # 2m UP (Z-axis is UP)
            transform.rotation = msg.pose.pose.orientation  # Same orientation!
            
            desired.transforms = [transform]
            desired.velocities = [Twist()]
            desired.accelerations = [Twist()]
            
            # Start publishing
            self.timer = self.create_timer(0.1, lambda: self.pub.publish(desired))
            
            self.get_logger().info(f'╔═══════════════════════════════════════════╗')
            self.get_logger().info(f'║         TAKEOFF COMMAND - 2M UP          ║')
            self.get_logger().info(f'╚═══════════════════════════════════════════╝')
            self.get_logger().info(f'Current Position:')
            self.get_logger().info(f'  X: {msg.pose.pose.position.x:.3f}')
            self.get_logger().info(f'  Y: {msg.pose.pose.position.y:.3f}')
            self.get_logger().info(f'  Z: {msg.pose.pose.position.z:.3f} (altitude)')
            self.get_logger().info(f'Target Position:')
            self.get_logger().info(f'  X: {transform.translation.x:.3f}')
            self.get_logger().info(f'  Y: {transform.translation.y:.3f}')
            self.get_logger().info(f'  Z: {transform.translation.z:.3f}')
            self.get_logger().info(f'Orientation: UNCHANGED')
            self.get_logger().info(f'Expected: Drone climbs smoothly to target altitude')

def main():
    rclpy.init()
    node = TakeoffPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()