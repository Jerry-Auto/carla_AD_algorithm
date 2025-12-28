#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import carla
import math


class RoadBoundaryMonitor(Node):
    def __init__(self):
        super().__init__('road_boundary_monitor')
        
        # 获取参数
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('publish_topic', '/carla/road_boundaries')
        self.declare_parameter('sticky_start_waypoint', True)
        self.declare_parameter('sticky_score_margin', 0.10)
        
        self.carla_host = self.get_parameter('carla_host').get_parameter_value().string_value
        self.carla_port = self.get_parameter('carla_port').get_parameter_value().integer_value
        self.role_name = self.get_parameter('role_name').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.sticky_start_waypoint = self.get_parameter('sticky_start_waypoint').get_parameter_value().bool_value
        self.sticky_score_margin = float(self.get_parameter('sticky_score_margin').get_parameter_value().double_value)

        self._last_start_wp_key = None
        
        # 初始化CARLA客户端
        self.client = carla.Client(self.carla_host, self.carla_port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        
        # 获取车辆
        self.vehicle = None
        self.get_vehicle()
        
        # 初始化发布者
        self.publisher_ = self.create_publisher(Float32MultiArray, self.publish_topic, 10)
        
        # 设置定时器（每秒发布一次）
        self.timer = self.create_timer(0.2, self.monitor_callback)
        
        self.get_logger().info(f'RoadBoundaryMonitor initialized. '
                              f'Listening to CARLA on {self.carla_host}:{self.carla_port}, '
                              f'role_name={self.role_name}')

    def get_lane_width_safe(self, waypoint: carla.Waypoint) -> float:
        try:
            return float(waypoint.lane_width)
        except RuntimeError:
            return 3.5

    def find_outermost_lane_id(self, road_id: int, s: float, direction: str) -> int:
        """找到同一侧（positive 或 negative）的最外侧 lane_id（对齐 test_road_width.py）。

        说明：这里按 lane_id 连续扫描，直到 get_waypoint_xodr 返回 None 为止。
        不做 lane_type 过滤，以避免某些地图在边缘 lane_type 变化导致过早截断。
        """
        if direction == 'positive':
            lane_id = 1
            last = 1
            guard = 0
            while guard < 64:
                wp = self.map.get_waypoint_xodr(road_id, lane_id, s)
                if wp is None:
                    break
                last = lane_id
                lane_id += 1
                guard += 1
            return last

        lane_id = -1
        last = -1
        guard = 0
        while guard < 64:
            wp = self.map.get_waypoint_xodr(road_id, lane_id, s)
            if wp is None:
                break
            last = lane_id
            lane_id -= 1
            guard += 1
        return last

    def distance_center_to_outer_edge(self, road_id: int, s: float, lane_from: int, lane_to: int, step: int) -> float:
        """从 lane_from 中心线到 lane_to 外侧边界的距离（对齐 test_road_width.py）。"""
        if lane_from == lane_to:
            wp = self.map.get_waypoint_xodr(road_id, lane_from, s)
            return self.get_lane_width_safe(wp) / 2.0 if wp is not None else 0.0

        wp_from = self.map.get_waypoint_xodr(road_id, lane_from, s)
        if wp_from is None:
            return 0.0

        distance = self.get_lane_width_safe(wp_from) / 2.0
        lane_id = lane_from + step
        guard = 0
        while guard < 64:
            wp = self.map.get_waypoint_xodr(road_id, lane_id, s)
            if wp is None:
                break
            distance += self.get_lane_width_safe(wp)
            if lane_id == lane_to:
                break
            lane_id += step
            guard += 1
        return distance

    def pick_next_waypoint(self, current_wp: carla.Waypoint, step_m: float):
        candidates = current_wp.next(step_m)
        if not candidates:
            return None

        same_lane = [wp for wp in candidates if wp.lane_id == current_wp.lane_id]
        if same_lane:
            same_road = [wp for wp in same_lane if wp.road_id == current_wp.road_id]
            return same_road[0] if same_road else same_lane[0]
        return candidates[0]

    def get_vehicle(self):
        """获取指定Role Name的车辆"""
        if self.vehicle:
            return True

        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                self.get_logger().info(f'Found vehicle with role_name: {self.role_name}, id: {actor.id}')
                return True
        return False

    def _yaw_to_forward(self, yaw_deg: float):
        yaw = math.radians(float(yaw_deg))
        return (math.cos(yaw), math.sin(yaw))

    def _dot2(self, a, b) -> float:
        return float(a[0] * b[0] + a[1] * b[1])

    def _wp_key(self, wp: carla.Waypoint):
        return (
            int(getattr(wp, 'road_id', 0)),
            int(getattr(wp, 'section_id', 0)),
            int(getattr(wp, 'lane_id', 0)),
        )

    def pick_waypoint_matching_heading(self, location: carla.Location, vehicle_transform: carla.Transform):
        """选择与车辆朝向最一致的 Driving waypoint。

        经验上，map.get_waypoint(location) 在路口/靠近中心线时可能投影到对向车道，
        导致左右宽度在两套结果间来回跳变。
        这里在左右相邻车道中搜索，选与车辆 forward vector 点积最大的同向车道。
        """
        base_wp = self.map.get_waypoint(
            location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if base_wp is None:
            return None

        v_fwd = self._yaw_to_forward(vehicle_transform.rotation.yaw)

        def wp_forward(wp: carla.Waypoint):
            tf = wp.transform
            return self._yaw_to_forward(tf.rotation.yaw)

        # 收集候选：base + 向左/向右扩展若干条车道
        candidates = []
        wp = base_wp
        for _ in range(8):
            if wp is None:
                break
            if wp.lane_type == carla.LaneType.Driving and int(getattr(wp, 'lane_id', 0)) != 0:
                candidates.append(wp)
            wp = wp.get_left_lane()

        wp = base_wp.get_right_lane()
        for _ in range(8):
            if wp is None:
                break
            if wp.lane_type == carla.LaneType.Driving and int(getattr(wp, 'lane_id', 0)) != 0:
                candidates.append(wp)
            wp = wp.get_right_lane()

        if not candidates:
            return base_wp

        # 先选同向（dot>0）的最优；若都不同向，则退化为 dot 最大
        best_wp = None
        best_dot = -1e9
        best_same = False
        scores = {}
        for wp in candidates:
            d = self._dot2(wp_forward(wp), v_fwd)
            scores[self._wp_key(wp)] = d
            same = d > 0.0
            if best_wp is None:
                best_wp, best_dot, best_same = wp, d, same
                continue
            if best_same:
                if same and d > best_dot:
                    best_wp, best_dot = wp, d
            else:
                # 当前最优还不是同向：只要发现同向就立即替换；否则比 dot
                if same or d > best_dot:
                    best_wp, best_dot, best_same = wp, d, same

        # 粘性/滞回：如果上一次选中的车道仍是“足够好”的候选，则保持不切换，避免在边界处抖动。
        if self.sticky_start_waypoint and self._last_start_wp_key is not None:
            last_key = self._last_start_wp_key
            if last_key in scores:
                last_dot = float(scores[last_key])
                # 只要上一次也是同向，并且与当前最优差距不大，就沿用上一次
                if last_dot > 0.0 and (best_dot - last_dot) < self.sticky_score_margin:
                    for wp in candidates:
                        if self._wp_key(wp) == last_key:
                            best_wp = wp
                            break

        self._last_start_wp_key = self._wp_key(best_wp) if best_wp is not None else None
        return best_wp

    def get_road_boundaries_same_direction(self, waypoint: carla.Waypoint):
        """获取同向车道的左右边界距离（对齐 test_road_width.py）。

        格式约定：左为正，右为负，均相对当前车道中心线。
        基于 get_waypoint_xodr(road_id, lane_id, s) 沿 lane_id 方向跨车道累加。
        """
        if waypoint is None:
            return 0.0, 0.0

        road_id = int(getattr(waypoint, 'road_id', 0))
        current_lane_id = int(getattr(waypoint, 'lane_id', 0))
        s = getattr(waypoint, 's', None)
        if s is None or current_lane_id == 0:
            return 0.0, 0.0

        if current_lane_id < 0:
            # 负 lane_id：同向车道通常为 -1,-2,...
            left_edge_lane = -1
            right_edge_lane = self.find_outermost_lane_id(road_id, s, direction='negative')
            left_distance = self.distance_center_to_outer_edge(road_id, s, current_lane_id, left_edge_lane, step=+1)
            right_distance = -self.distance_center_to_outer_edge(road_id, s, current_lane_id, right_edge_lane, step=-1)
            return float(left_distance), float(right_distance)

        # 正 lane_id：同向车道通常为 1,2,...
        left_edge_lane = 1
        right_edge_lane = self.find_outermost_lane_id(road_id, s, direction='positive')
        left_distance = self.distance_center_to_outer_edge(road_id, s, current_lane_id, left_edge_lane, step=-1)
        right_distance = -self.distance_center_to_outer_edge(road_id, s, current_lane_id, right_edge_lane, step=+1)
        return float(left_distance), float(right_distance)

    def monitor_callback(self):
        """主监控回调函数"""
        if not self.vehicle:
            if not self.get_vehicle():
                # 车辆未找到，等待下一次回调
                return
        
        # 获取车辆位置与姿态
        vehicle_tf = self.vehicle.get_transform()
        location = vehicle_tf.location

        # 获取当前 waypoint（匹配车辆朝向，避免投影到对向车道导致左右跳变）
        start_waypoint = self.pick_waypoint_matching_heading(location, vehicle_tf)
        
        if not start_waypoint:
            return

        # 采样参数（与 test_road_width.py 一致：起点 + 向前 300m，每 1m 一个点）
        lookahead_dist = 300.0
        step_dist = 1.0
        num_points = int(lookahead_dist / step_dist) + 1
        
        data_list = [step_dist] # 第一个元素是分辨率
        
        current_wp = start_waypoint
        
        for _ in range(num_points):
            # 获取当前点的边界
            left, right = self.get_road_boundaries_same_direction(current_wp)
            data_list.append(left)
            data_list.append(right)
            
            # 获取下一个点
            next_wp = self.pick_next_waypoint(current_wp, step_dist)
            if next_wp is None:
                break
            current_wp = next_wp
            
        # 创建并发布消息
        msg = Float32MultiArray()
        msg.data = data_list
        
        self.publisher_.publish(msg)
        # self.get_logger().debug(f'Published road boundaries for {len(data_list)//2} points')

def main(args=None):
    rclpy.init(args=args)
    node = RoadBoundaryMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()