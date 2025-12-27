
import sys
import time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

def main():
    rclpy.init()
    node = Node("debug_sub")
    
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.VOLATILE)
    
    def cb(msg):
        print("Received msg!", flush=True)
        
    node.create_subscription(Odometry, "/carla/ego_vehicle/odometry", cb, qos)
    
    print("Spinning...", flush=True)
    rclpy.spin(node)

if __name__ == "__main__":
    main()
