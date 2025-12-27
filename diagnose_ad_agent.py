import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from carla_msgs.msg import CarlaEgoVehicleControl

class DiagnoseAdAgent(Node):
    def __init__(self):
        super().__init__('diagnose_ad_agent')
        
        self.target_speed_sub = self.create_subscription(
            Float64,
            '/carla/ego_vehicle/target_speed',
            self.target_speed_callback,
            10)
        
        self.speed_command_sub = self.create_subscription(
            Float64,
            '/carla/ego_vehicle/speed_command',
            self.speed_command_callback,
            10)
            
        self.waypoints_sub = self.create_subscription(
            Path,
            '/carla/ego_vehicle/waypoints',
            self.waypoints_callback,
            10)
            
        self.control_cmd_sub = self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd',
            self.control_cmd_callback,
            10)

        self.odometry_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10)

        self.timer = self.create_timer(2.0, self.print_status)
        
        self.last_target_speed = None
        self.last_speed_command = None
        self.last_waypoints = None
        self.last_control_cmd = None
        self.last_odometry = None

    def target_speed_callback(self, msg):
        self.last_target_speed = msg.data

    def speed_command_callback(self, msg):
        self.last_speed_command = msg.data

    def waypoints_callback(self, msg):
        self.last_waypoints = len(msg.poses)

    def control_cmd_callback(self, msg):
        self.last_control_cmd = msg

    def odometry_callback(self, msg):
        self.last_odometry = msg

    def print_status(self):
        print("-" * 30)
        print(f"Target Speed: {self.last_target_speed}")
        print(f"Speed Command: {self.last_speed_command}")
        print(f"Waypoints Count: {self.last_waypoints}")
        if self.last_control_cmd:
            print(f"Control Cmd: Throttle={self.last_control_cmd.throttle}, Brake={self.last_control_cmd.brake}, Steer={self.last_control_cmd.steer}")
        else:
            print("Control Cmd: None")
        
        if self.last_odometry:
             print(f"Odometry: Received")
        else:
             print(f"Odometry: None")
        print("-" * 30)

def main(args=None):
    rclpy.init(args=args)
    node = DiagnoseAdAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
