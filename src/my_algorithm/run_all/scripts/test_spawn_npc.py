#!/usr/bin/env python3
import carla
import random
import time

def main():
    host = 'localhost'
    port = 2000

    print(f"Connecting to CARLA at {host}:{port}...")
    try:
        client = carla.Client(host, port)
        client.set_timeout(5.0)
        world = client.get_world()
        print("Connected successfully.")
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Get blueprint library
    bp_lib = world.get_blueprint_library()
    vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
    
    if not vehicle_bps:
        print("No vehicle blueprints found.")
        return

    bp = random.choice(vehicle_bps)
    if bp.has_attribute('color'):
        color = random.choice(bp.get_attribute('color').recommended_values)
        bp.set_attribute('color', color)

    # Get a spawn point
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        print("No spawn points found in map.")
        # Fallback to a hardcoded point if map has no spawn points (unlikely for standard maps)
        transform = carla.Transform(carla.Location(x=0, y=0, z=5), carla.Rotation(0,0,0))
    else:
        transform = random.choice(spawn_points)

    print(f"Attempting to spawn {bp.id} at {transform.location}")

    try:
        vehicle = world.spawn_actor(bp, transform)
        print(f"Spawned vehicle: {vehicle.id}")
        
        try:
            # Try to enable autopilot
            vehicle.set_autopilot(True)
            print("Autopilot enabled.")
        except Exception as e:
            print(f"Autopilot failed: {e}")
            # Don't destroy vehicle if autopilot fails, just to see if it spawned
        
        # Keep it alive for a few seconds to verify
        time.sleep(2)
        print("Test successful.")
        
    except Exception as e:
        print(f"Spawn failed: {e}")

if __name__ == '__main__':
    main()
