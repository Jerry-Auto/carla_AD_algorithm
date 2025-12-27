
import carla
import time

def check_spawn_point():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    
    try:
        world = client.get_world()
        m = world.get_map()
        if m.name.split('/')[-1] != 'Town04':
            print(f"Current map is {m.name}, loading Town04...")
            client.load_world('Town04')
            world = client.get_world()
            m = world.get_map()
            
        # Target location from xosc
        loc = carla.Location(x=-9.4, y=-152.8, z=2.0)
        wp = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Driving)
        
        print(f"Target Location: {loc}")
        print(f"Nearest Waypoint: {wp.transform.location}")
        print(f"Distance to road: {loc.distance(wp.transform.location)}")
        
        # Try to spawn a vehicle there
        bp_lib = world.get_blueprint_library()
        bp = bp_lib.find('vehicle.tesla.model3')
        
        spawn_transform = carla.Transform(loc, carla.Rotation(yaw=90)) # h=1.57 rad is approx 90 deg
        
        print(f"Attempting to spawn at {spawn_transform}")
        
        vehicle = world.try_spawn_actor(bp, spawn_transform)
        
        if vehicle:
            print("Spawn SUCCESS!")
            time.sleep(2)
            print(f"Vehicle location after 2s: {vehicle.get_location()}")
            vehicle.destroy()
        else:
            print("Spawn FAILED! Collision detected?")
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    check_spawn_point()
