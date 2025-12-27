
import sys
import os
import time
import threading

# Add paths to sys.path
sys.path.append('/home/zhangjinrui/AppDisk/My_Code_Dir/ros_simulation/carla_AD_algorithm/src/ros2_bridge/ros_compatibility/src')
sys.path.append('/home/zhangjinrui/AppDisk/My_Code_Dir/ros_simulation/carla_AD_algorithm/src/ros2_bridge/carla_ad_agent/src')

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Odometry

class TestAgent(CompatibleNode):
    def __init__(self):
        super(TestAgent, self).__init__("test_agent")
        role_name = "ego_vehicle"
        
        self._odometry_subscriber = self.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(role_name),
            self.odometry_cb,
            qos_profile=10,
            callback_group=None
        )
        self.loginfo("Subscribed to odometry topic: /carla/{}/odometry".format(role_name))
        
        self.update_timer = self.new_timer(
            0.05, lambda timer_event=None: self.run_step())

    def odometry_cb(self, odometry_msg):
        self.loginfo("Received odometry message")
        print("Received odometry message", flush=True)

    def run_step(self):
        self.loginfo("run_step")
        print("run_step", flush=True)

def main(args=None):
    roscomp.init("test_agent", args=args)
    try:
        executor = roscomp.executors.MultiThreadedExecutor()
        controller = TestAgent()
        executor.add_node(controller)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

if __name__ == "__main__":
    main()
