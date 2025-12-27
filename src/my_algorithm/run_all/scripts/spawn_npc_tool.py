#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PointStamped
import carla
import random
import sys

class SpawnNpcTool(Node):
    def __init__(self):
        super().__init__('spawn_npc_tool')
        
        # Parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        
        self.host = self.get_parameter('host').value
        self.port = self.get_parameter('port').value
        
        self.world = None
        self.connect_to_carla()
        
        # Subscriber
        # Use a standard QoS profile compatible with RViz
        # RViz usually publishes /clicked_point as Reliable, Volatile
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.listener_callback,
            10) # Use integer depth for default QoS, which is usually safest
        self.get_logger().info("Spawn NPC Tool initialized. Use 'Publish Point' in RViz to spawn vehicles.")
        
        # Heartbeat timer to check connection
        self.create_timer(5.0, self.heartbeat_callback)

    def heartbeat_callback(self):
        if not self.world:
            self.get_logger().warn("CARLA connection lost or not established. Retrying...")
            self.connect_to_carla()

    def connect_to_carla(self):
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(2.0)
            self.world = client.get_world()
            self.get_logger().info(f"Connected to CARLA Server at {self.host}:{self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received click at ROS(x={msg.point.x:.1f}, y={msg.point.y:.1f}, z={msg.point.z:.1f})")
        
        if not self.world:
            self.get_logger().warn("Cannot spawn: No connection to CARLA world.")
            self.connect_to_carla()
            if not self.world:
                return

        # Coordinate conversion: ROS (ENU) -> CARLA (Left-handed)
        x = msg.point.x
        y = -msg.point.y
        z = msg.point.z
        
        # Try to snap to the nearest road
        try:
            location = carla.Location(x=x, y=y, z=z)
            # Increase search distance to find road
            waypoint = self.world.get_map().get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
            
            if waypoint:
                transform = waypoint.transform
                transform.location.z += 0.5 # Lift slightly to avoid collision with ground
                self.get_logger().info(f"Snapped to road at CARLA(x={transform.location.x:.1f}, y={transform.location.y:.1f})")
            else:
                self.get_logger().warn("Could not snap to road, using raw coordinates with offset")
                transform = carla.Transform(carla.Location(x=x, y=y, z=z + 2.0), carla.Rotation(yaw=random.uniform(0, 360)))
        except Exception as e:
             self.get_logger().warn(f"Error finding waypoint: {e}")
             transform = carla.Transform(carla.Location(x=x, y=y, z=z + 2.0), carla.Rotation(yaw=random.uniform(0, 360)))
        
        # Pick a random vehicle blueprint
        bp_lib = self.world.get_blueprint_library()
        vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
        
        if not vehicle_bps:
            self.get_logger().warn("No vehicle blueprints found!")
            return
            
        bp = random.choice(vehicle_bps)
        
        # Color
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Spawn
        vehicle = None
        try:
            vehicle = self.world.spawn_actor(bp, transform)
            self.get_logger().info(f"Spawned {bp.id} successfully (ID: {vehicle.id})")
        except Exception as e:
            self.get_logger().warn(f"Failed to spawn actor: {e}")
            return

        # Set Autopilot or Static
        if vehicle:
            try:
                if random.random() < 0.5:
                    # 50% chance: Static (No Autopilot)
                    self.get_logger().info("Vehicle spawned as STATIC (No Autopilot)")
                else:
                    # 50% chance: Autopilot
                    # Try to use a random port for TM to avoid bind error if default 8000 is taken
                    tm_port = 8000
                    vehicle.set_autopilot(True, tm_port)
                    self.get_logger().info("Vehicle spawned with AUTOPILOT enabled")
            except Exception as e:
                self.get_logger().warn(f"Failed to set vehicle mode: {e}")
                # Fallback logic for autopilot if needed...
                if not vehicle.get_autopilot_status(): # Check if it failed to set
                     try:
                        tm_port = random.randint(8005, 8100)
                        vehicle.set_autopilot(True, tm_port)
                        self.get_logger().info(f"Autopilot enabled on fallback port {tm_port}")
                     except:
                        pass

def main(args=None):
    rclpy.init(args=args)
    node = SpawnNpcTool()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
