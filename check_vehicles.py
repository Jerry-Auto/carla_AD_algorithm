
import carla
import os

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    
    vehicles = world.get_actors().filter('vehicle.*')
    print(f"Found {len(vehicles)} vehicles.")
    
    for v in vehicles:
        role_name = v.attributes.get('role_name', 'N/A')
        print(f"Vehicle ID: {v.id}, Type: {v.type_id}, Role Name: '{role_name}'")

if __name__ == '__main__':
    main()
