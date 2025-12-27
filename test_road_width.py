#!/usr/bin/env python3
import carla
import argparse
import sys


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Standalone CARLA road width test. "
            "By default, uses ego vehicle location if found; otherwise falls back to a spawn point."
        )
    )
    parser.add_argument('--host', default='localhost', help='CARLA host (default: localhost)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA port (default: 2000)')

    group = parser.add_mutually_exclusive_group()
    group.add_argument('--use-ego', action='store_true', help='Use ego vehicle location (role_name)')
    group.add_argument('--spawn-index', type=int, help='Use a specific spawn point index')
    group.add_argument('--xyz', nargs=3, type=float, metavar=('X', 'Y', 'Z'), help='Use a manual location')

    parser.add_argument('--role-name', default='ego_vehicle', help="Ego vehicle role_name (default: 'ego_vehicle')")

    parser.add_argument('--forward', type=float, default=300.0, help='Scan distance forward in meters (default: 300)')
    parser.add_argument('--step', type=float, default=1.0, help='Sampling interval in meters (default: 1)')
    return parser.parse_args()


def find_vehicle_by_role(world: carla.World, role_name: str):
    actors = world.get_actors().filter('vehicle.*')
    for actor in actors:
        try:
            if actor.attributes.get('role_name') == role_name:
                return actor
        except Exception:
            continue
    return None


def pick_next_waypoint(current_wp: carla.Waypoint, step_m: float):
    """Pick a deterministic next waypoint along the lane centerline."""
    candidates = current_wp.next(step_m)
    if not candidates:
        return None

    # Prefer staying on same lane_id (and usually same road_id), otherwise fall back to the first.
    same_lane = [wp for wp in candidates if wp.lane_id == current_wp.lane_id]
    if same_lane:
        same_road = [wp for wp in same_lane if wp.road_id == current_wp.road_id]
        return same_road[0] if same_road else same_lane[0]
    return candidates[0]


def scan_forward_boundaries(map: carla.Map, start_wp: carla.Waypoint, forward_m: float, step_m: float):
    """From start waypoint, walk forward along the waypoint path and compute boundaries at each sample."""
    if step_m <= 0:
        raise ValueError('step must be > 0')
    if forward_m < 0:
        raise ValueError('forward must be >= 0')

    samples = []
    current_wp = start_wp
    traveled = 0.0
    max_steps = int(forward_m / step_m) + 1

    for i in range(max_steps):
        left, right = get_road_boundaries_same_direction(map, current_wp)
        samples.append(
            {
                'i': i,
                's_rel': traveled,
                'x': float(current_wp.transform.location.x),
                'y': float(current_wp.transform.location.y),
                'z': float(current_wp.transform.location.z),
                'road_id': int(current_wp.road_id),
                'section_id': int(current_wp.section_id),
                'lane_id': int(current_wp.lane_id),
                'left': float(left),
                'right': float(right),
            }
        )

        if traveled + step_m > forward_m:
            break

        next_wp = pick_next_waypoint(current_wp, step_m)
        if next_wp is None:
            break
        current_wp = next_wp
        traveled += step_m

    return samples

def main():
    args = parse_args()

    # 连接到 CARLA
    print("Connecting to CARLA...")
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()
    
    if not map:
        print("ERROR: Failed to get map from CARLA!")
        sys.exit(1)
    
    print("Connected to CARLA. Map loaded.")
    
    # 获取所有可用的 spawn points
    spawn_points = world.get_map().get_spawn_points()
    print(f"Found {len(spawn_points)} spawn points in the map.")

    # 选择测试点：优先级
    # 1) --xyz
    # 2) --spawn-index
    # 3) 默认/--use-ego：ego vehicle
    # 4) fallback：第一个有效 spawn point
    test_point = None

    if args.xyz is not None:
        x, y, z = args.xyz
        test_point = carla.Location(x=x, y=y, z=z)
        print(f"Using manual location (--xyz): {test_point}")
    elif args.spawn_index is not None:
        if not spawn_points:
            print("ERROR: Map has no spawn points")
            sys.exit(1)
        idx = max(0, min(args.spawn_index, len(spawn_points) - 1))
        test_point = spawn_points[idx].location
        print(f"Using spawn point index {idx}: {test_point}")
    else:
        # 默认：尝试用 ego 车
        vehicle = find_vehicle_by_role(world, args.role_name)
        if vehicle is not None:
            test_point = vehicle.get_transform().location
            print(f"Using ego vehicle '{args.role_name}' location: {test_point}")
        else:
            print(f"Ego vehicle '{args.role_name}' not found; falling back to spawn point")

    if test_point is None:
        # fallback：选择第一个在道路上的 spawn point
        for spawn in spawn_points:
            waypoint = map.get_waypoint(spawn.location, project_to_road=True)
            if waypoint:
                test_point = spawn.location
                print(f"Using fallback spawn point: {test_point}")
                break

    if test_point is None:
        print("ERROR: No valid spawn points found on the road!")
        test_point = carla.Location(x=150.0, y=150.0, z=0.0)
        print(f"Using last-resort fallback location: {test_point}")
    
    # 获取测试点的路点（以 waypoint 全局路径/中心线为中心）
    waypoint = map.get_waypoint(test_point, project_to_road=True, lane_type=carla.LaneType.Driving)
    
    if not waypoint:
        print(f"ERROR: Could not get waypoint at {test_point}")
        print("Available spawn points:")
        for i, spawn in enumerate(spawn_points):
            print(f"  Spawn point {i}: {spawn.location}")
        sys.exit(1)
    
    print(f"\nStart location: {test_point}")
    print(f"Start waypoint at: {waypoint.transform.location}")
    print(f"Start Road ID: {waypoint.road_id}, Section ID: {waypoint.section_id}, Lane ID: {waypoint.lane_id}")
    
    # 重要：CARLA Python API 的 get_waypoint_xodr 签名为 (road_id, lane_id, s)
    # section_id 不能作为参数传入，否则会导致取错 lane 或触发内部错误。
    s = getattr(waypoint, 's', None)
    if s is None:
        print("ERROR: waypoint.s is missing; cannot use get_waypoint_xodr reliably")
        sys.exit(1)

    # 从车辆当前位置起点，沿 waypoint 全局路径中心线向前采样
    samples = scan_forward_boundaries(map, waypoint, args.forward, args.step)
    if not samples:
        print('ERROR: no samples generated')
        sys.exit(1)

    print(f"\nScan forward: {samples[-1]['s_rel']:.1f}m (requested {args.forward:.1f}m), step={args.step:.1f}m, N={len(samples)}")
    print(f"{'i':<5} {'s(m)':<7} {'road':<6} {'lane':<6} {'left(m)':<10} {'right(m)':<10} {'x':<10} {'y':<10}")
    print('-' * 70)

    # 只打印少量行，避免刷屏：前 5 行 + 后 5 行
    head_n = min(5, len(samples))
    tail_n = min(5, max(0, len(samples) - head_n))

    def print_row(row):
        print(
            f"{row['i']:<5d} {row['s_rel']:<7.1f} {row['road_id']:<6d} {row['lane_id']:<6d} "
            f"{row['left']:<10.2f} {row['right']:<10.2f} {row['x']:<10.2f} {row['y']:<10.2f}"
        )

    for r in samples[:head_n]:
        print_row(r)
    if len(samples) > head_n + tail_n:
        print('...')
    for r in samples[-tail_n:]:
        print_row(r)

    left_min = min(r['left'] for r in samples)
    left_max = max(r['left'] for r in samples)
    right_min = min(r['right'] for r in samples)
    right_max = max(r['right'] for r in samples)

    print('\nSummary (same-direction boundaries, centered on waypoint path):')
    print(f"left:  min={left_min:.2f}m, max={left_max:.2f}m")
    print(f"right: min={right_min:.2f}m, max={right_max:.2f}m")

def get_total_road_width_all(map, road_id, s):
    """同一 road_id、同一 s 处：所有 Driving 车道宽度之和（包含双向）"""
    total_width = 0.0
    
    # 向左遍历（正数车道ID）
    lane_id = 1
    while True:
        wp = map.get_waypoint_xodr(road_id, lane_id, s)
        if wp is None:
            break
        total_width += get_lane_width_safe(wp)
        lane_id += 1
    
    # 向右遍历（负数车道ID）
    lane_id = -1
    while True:
        wp = map.get_waypoint_xodr(road_id, lane_id, s)
        if wp is None:
            break
        total_width += get_lane_width_safe(wp)
        lane_id -= 1
    
    return total_width


def get_total_road_width_same_direction(map, road_id, s, current_lane_id):
    """同一 road_id、同一 s 处：仅统计与 current_lane_id 同向（同号）的 Driving 车道宽度之和"""
    if current_lane_id == 0:
        return 0.0

    sign = 1 if current_lane_id > 0 else -1
    total_width = 0.0

    if sign > 0:
        lane_id = 1
        while True:
            wp = map.get_waypoint_xodr(road_id, lane_id, s)
            if wp is None:
                break
            total_width += get_lane_width_safe(wp)
            lane_id += 1
    else:
        lane_id = -1
        while True:
            wp = map.get_waypoint_xodr(road_id, lane_id, s)
            if wp is None:
                break
            total_width += get_lane_width_safe(wp)
            lane_id -= 1

    return total_width

def get_lane_width_safe(waypoint):
    """安全获取车道宽度（避免触发断言错误）"""
    # 使用 CARLA 内部 API 获取车道宽度
    try:
        # 这是 CARLA 0.9.13+ 的安全方法
        return waypoint.lane_width
    except RuntimeError:
        # 如果失败，返回默认值
        return 3.5

def get_road_boundaries_same_direction(map, waypoint):
    """获取同向车道的左右边界距离（左为正，右为负，均相对当前车道中心线）"""
    road_id = waypoint.road_id
    current_lane_id = waypoint.lane_id
    s = getattr(waypoint, 's', None)
    if s is None or current_lane_id == 0:
        return 0.0, 0.0

    if current_lane_id < 0:
        # 负 lane_id：同向车道为 -1,-2,...（通常是顺着 road 参考方向）
        left_edge_lane = -1
        right_edge_lane = find_outermost_lane_id(map, road_id, s, direction='negative')
        left_distance = distance_center_to_outer_edge(map, road_id, s, current_lane_id, left_edge_lane, step=+1)
        right_distance = -distance_center_to_outer_edge(map, road_id, s, current_lane_id, right_edge_lane, step=-1)
        return left_distance, right_distance

    # 正 lane_id：同向车道为 1,2,...（与 road 参考方向相反）
    left_edge_lane = 1
    right_edge_lane = find_outermost_lane_id(map, road_id, s, direction='positive')
    left_distance = distance_center_to_outer_edge(map, road_id, s, current_lane_id, left_edge_lane, step=-1)
    right_distance = -distance_center_to_outer_edge(map, road_id, s, current_lane_id, right_edge_lane, step=+1)
    return left_distance, right_distance


def find_outermost_lane_id(map, road_id, s, direction: str):
    """找到同一侧（positive 或 negative）Driving 车道的最外侧 lane_id。"""
    if direction == 'positive':
        lane_id = 1
        last = 1
        while True:
            wp = map.get_waypoint_xodr(road_id, lane_id, s)
            if wp is None:
                break
            last = lane_id
            lane_id += 1
        return last

    # negative
    lane_id = -1
    last = -1
    while True:
        wp = map.get_waypoint_xodr(road_id, lane_id, s)
        if wp is None:
            break
        last = lane_id
        lane_id -= 1
    return last


def distance_center_to_outer_edge(map, road_id, s, lane_from, lane_to, step: int):
    """从 lane_from 的中心线出发，沿着 lane_id 方向 step 逐车道跨越，直到 lane_to 的外侧边界的距离。"""
    if lane_from == lane_to:
        wp = map.get_waypoint_xodr(road_id, lane_from, s)
        return get_lane_width_safe(wp) / 2.0 if wp is not None else 0.0

    wp_from = map.get_waypoint_xodr(road_id, lane_from, s)
    if wp_from is None:
        return 0.0

    distance = get_lane_width_safe(wp_from) / 2.0
    lane_id = lane_from + step
    guard = 0
    while guard < 64:
        wp = map.get_waypoint_xodr(road_id, lane_id, s)
        if wp is None:
            break
        w = get_lane_width_safe(wp)
        distance += w
        if lane_id == lane_to:
            break
        lane_id += step
        guard += 1
    return distance

def get_distance_to_left_edge(map, waypoint):
    """兼容旧接口：返回同向左边界距离（左为正）"""
    left, _ = get_road_boundaries_same_direction(map, waypoint)
    return left

def get_distance_to_right_edge(map, waypoint):
    """兼容旧接口：返回同向右边界“绝对距离”（右为正值），用于验证输出"""
    _, right = get_road_boundaries_same_direction(map, waypoint)
    return abs(right)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"ERROR: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)