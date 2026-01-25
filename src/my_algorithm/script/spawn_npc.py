#!/usr/bin/env python3
"""
基于自车车道的前向交通流生成：
- 在自车前方 500m 到 800m 的道路上随机挑选车道生成 NPC
- 按距离间隔（默认 10m）生成
- 远离自车 1000m 自动销毁
"""

import random
import time
import carla

# ====== 配置区：可按需调整 ======
SPAWN_AHEAD_DISTANCE_MAX = 800.0   # 自车前方npc生成点范围上限（m）
SPAWN_AHEAD_DISTANCE_MIN = 500.0   # 自车前方npc生成点范围下限（m）
DESPAWN_DISTANCE = 1000.0      # 远离自车超过该距离自动销毁（m）
SPAWN_GAP_MIN = 20            # 生成间隔最小值（m）
SPAWN_GAP_MAX = 30          # 生成间隔最大值（m）
SPAWN_CHECK_INTERVAL = 0.1    # 检查/生成周期（s）
NPC_SPEED_MIN = 20.0           # NPC 最小速度（km/h）
NPC_SPEED_MAX = 60.0           # NPC 最大速度（km/h）
USE_TRAJECTORY = False         # 是否使用预录制轨迹控制 NPC
TRAJECTORY_FILE = "./traffic_stream_data/01_tracks.csv"  # 轨迹文件路径（CSV 格式：x,y,z,yaw）# NPC_BP_IDS 将动态获取所有可用车辆蓝图
# ==================================


def get_ego_vehicle(world):
    vehicles = world.get_actors().filter("vehicle.*")
    if not vehicles:
        return None
    for v in vehicles:
        if v.attributes.get("role_name") == "hero" or v.attributes.get("role_name") == "ego_vehicle":
            return v
    return vehicles[0]


def collect_driving_lanes(waypoint):
    lanes = [waypoint]

    left = waypoint.get_left_lane()
    while left and left.lane_type == carla.LaneType.Driving:
        lanes.append(left)
        left = left.get_left_lane()

    right = waypoint.get_right_lane()
    while right and right.lane_type == carla.LaneType.Driving:
        lanes.append(right)
        right = right.get_right_lane()

    return lanes


def main():
    global USE_TRAJECTORY
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    carla_map = world.get_map()
    traffic_manager = client.get_trafficmanager(8000)

    blueprint_lib = world.get_blueprint_library()
    all_vehicle_bps = blueprint_lib.filter("vehicle.*")
    NPC_BP_IDS = [bp.id for bp in all_vehicle_bps]

    trajectory_waypoints = {}
    if USE_TRAJECTORY:
        import csv
        import math
        try:
            with open(TRAJECTORY_FILE, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    vehicle_id = int(row['id'])
                    x = float(row['x'])
                    y = float(row['y'])
                    x_vel = float(row['xVelocity'])
                    y_vel = float(row['yVelocity'])
                    yaw = math.degrees(math.atan2(y_vel, x_vel)) if x_vel != 0 or y_vel != 0 else 0.0
                    loc = carla.Location(x=x, y=y, z=0.0)
                    if vehicle_id not in trajectory_waypoints:
                        trajectory_waypoints[vehicle_id] = []
                    trajectory_waypoints[vehicle_id].append(loc)
            print(f"📍 Loaded trajectories for {len(trajectory_waypoints)} vehicles.")
        except FileNotFoundError:
            print(f"❌ Trajectory file {TRAJECTORY_FILE} not found. Using autopilot.")
            USE_TRAJECTORY = False

    print(f"🚗 Loaded {len(NPC_BP_IDS)} vehicle blueprints for spawning.")

    spawned = []
    last_spawn_location = None
    prev_ego_location = None
    distance_since_last_spawn = random.uniform(SPAWN_GAP_MIN, SPAWN_GAP_MAX)
    current_spawn_gap = distance_since_last_spawn  # 初始值

    print("🚗 Traffic flow spawner started. Waiting for ego vehicle...")

    try:
        while True:
            world.wait_for_tick()

            ego = get_ego_vehicle(world)
            if ego is None:
                time.sleep(SPAWN_CHECK_INTERVAL)
                continue

            ego_location = ego.get_location()
            if prev_ego_location is not None:
                distance_since_last_spawn += ego_location.distance(prev_ego_location)
            prev_ego_location = ego_location
            ego_wp = carla_map.get_waypoint(
                ego_location,
                project_to_road=True,
                lane_type=carla.LaneType.Driving,
            )
            if ego_wp is None:
                time.sleep(SPAWN_CHECK_INTERVAL)
                continue

            # 清理远离自车的 NPC
            alive_spawned = []
            for v in spawned:
                if v.is_alive:
                    dist = v.get_location().distance(ego_location)
                    if dist > DESPAWN_DISTANCE:
                        v.destroy()
                    else:
                        alive_spawned.append(v)
            spawned = alive_spawned

            # 按距离间隔生成
            if distance_since_last_spawn >= current_spawn_gap:
                next_wps = ego_wp.next(min(SPAWN_AHEAD_DISTANCE_MAX, SPAWN_AHEAD_DISTANCE_MIN))
                if next_wps:
                    target_wp = next_wps[0]
                    if ego_location.distance(target_wp.transform.location) <= SPAWN_AHEAD_DISTANCE_MAX:
                        lane_candidates = collect_driving_lanes(target_wp)
                        chosen_wp = random.choice(lane_candidates)

                        road_loc = chosen_wp.transform.location
                        safe_z = road_loc.z + 0.02
                        transform = carla.Transform(
                            carla.Location(x=road_loc.x, y=road_loc.y, z=safe_z),
                            chosen_wp.transform.rotation,
                        )

                        vehicle_bp = blueprint_lib.find(random.choice(NPC_BP_IDS))
                        if vehicle_bp.has_attribute("color"):
                            # 随机生成颜色以增加多样性
                            r = random.randint(0, 255)
                            g = random.randint(0, 255)
                            b = random.randint(0, 255)
                            vehicle_bp.set_attribute("color", f"{r},{g},{b}")

                        too_close = False
                        for v in spawned:
                            if v.is_alive and v.get_location().distance(transform.location) < current_spawn_gap * 0.6:
                                too_close = True
                                break

                        if not too_close:
                            try:
                                vehicle = world.spawn_actor(vehicle_bp, transform)
                                vehicle.set_autopilot(True, traffic_manager.get_port())
                                if USE_TRAJECTORY and trajectory_waypoints:
                                    selected_id = random.choice(list(trajectory_waypoints.keys()))
                                    traffic_manager.set_path(vehicle, trajectory_waypoints[selected_id])
                                else:
                                    desired_speed = random.uniform(NPC_SPEED_MIN, NPC_SPEED_MAX)
                                    traffic_manager.set_desired_speed(vehicle, desired_speed)
                                spawned.append(vehicle)
                                last_spawn_location = vehicle.get_location()
                                distance_since_last_spawn = 0.0
                                current_spawn_gap = random.uniform(SPAWN_GAP_MIN, SPAWN_GAP_MAX)
                                print(
                                    f"✅ Spawned NPC at {last_spawn_location.x:.1f}, "
                                    f"{last_spawn_location.y:.1f} (lane {chosen_wp.lane_id})"
                                )
                            except RuntimeError:
                                pass

            time.sleep(SPAWN_CHECK_INTERVAL)

    except KeyboardInterrupt:
        print("\n🛑 Stop requested. Cleaning up...")
    finally:
        for v in spawned:
            if v.is_alive:
                v.destroy()
        print("🧹 Done.")

if __name__ == '__main__':
    main()