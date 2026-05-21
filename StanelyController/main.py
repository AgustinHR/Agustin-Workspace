#
#   Name: Agustin Hernandez Reynoso
#   Course:
#   Project: Geometric Controllers Performance 
#   Description: Implementing a Stanely controller 
#               
#   Date: 5/15/26
#   Important Notes:

#       Carla Documentation: https://carla.readthedocs.io/en/latest/foundations/
#  
#       Only do the following in low memory computers
#           This implementation uses CARLA to run our simulations. The simulation enviorment is 
#           graphically heavy and will crash if GPU does not have enough memory. To prevent simulation
#           from crashing open the simulation with the following command 
#   
#           Command: .\CarlaUE4.exe -quality-level=Low -windowed -ResX=640 -ResY=480 -dx11
#       
#       Methods
#       - get_specator(): acts as the camera and controls the view of the simulation window
#       - get_blueprint_library(): returns a list of all aviable objects that can be used in simulation
#       - get_spawn_points(): returns list of recommended spawn points (x,y,z,roll,pitch,yaw) for the vehicle
#

################################# Packages ######################################
import sys
import carla 
import time

# Replace with where you have PythonAPI/carla located in your computer
sys.path.insert(1,'C:\\Users\\ejahi\\carla\\PythonAPI\\carla')

from agents.navigation.global_route_planner import GlobalRoutePlanner

##################### Provided by CARLA Documenation ###########################
def visualize_spawn_points_on_map(world, spawnPoints):
    for i, spawn_point in enumerate(spawnPoints):
        # Draw in the spectator window the spawn point index
        world.debug.draw_string(spawn_point.location, str(i), life_time=100)
        # We can also draw an arrow to see the orientation of the spawn point
        # (i.e. which way the vehicle will be facing when spawned)
        world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

################################# Custom #######################################

class PIDControl:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.accumulatedError = 0
        self.previousError = 0

    def control(self, vref, v, dt):
        error = vref - v
        self.accumulatedError += error * dt
    
        porportionalTerm = self.Kp * error
        integralTerm = self.Ki * self.accumulatedError
        derivativeTerm = self.Kp * ( (error - self.previousError) / dt )

        self.previousError = error 

        return porportionalTerm + integralTerm + derivativeTerm

class stanleyController:
    def __init(self):
        pass

def intialize_specator(world,vehicle):
    try:
        spectator = world.get_spectator()
        vehicle_current_location = vehicle.get_transform()
        adjusting_position = vehicle_current_location.location + carla.Location(y=7,z=2) # Adjust if camera position not over car
        adjusting_oreintation = carla.Rotation(0,-90,0) # Adjust if camera orientation is wrong [pitch,yaw,row]
        spectator.set_transform(carla.Transform(adjusting_position, adjusting_oreintation))
    except:
        print("Error: Something went wrong when trying to initialize spectator")

def create_way_points(world,planner,start,end):
    path = planner.trace_route(carla.Location(start.location), carla.Location(end.location))
    waypoints = []
    for w in path:
        waypoints.append(w[0].transform.location)
    return waypoints

def draw_way_points(world,waypoints):
    for w in waypoints:
        world.debug.draw_string(w, 
                                'O', draw_shadow=False,
                                color = carla.Color(r=0, g=0, b=255), 
                                life_time=1000.0,
                                persistent_lines=True)
        
################################## Main ########################################
simulationTime = 20 
startPoint = None
endPoint = None
sampleResolutoin = 2

def main():

    client = None
    world = None
    vehicle = None 
    map = None
    waypoints = []

    try:
        # Lets connect to the Carla server
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.00)
        print("Connecting to Carla server")

        # Create world 
        world = client.get_world()
        client.load_world('Town04')
        map = world.get_map()
        print(f"Connecting to world {map.name}")

        # # Spawn points on the current map
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points found")
            return 
        startPoint = spawn_points[50]
        endPoint = spawn_points[364]

        # # Choosing Vehicle
        blueprint_library = world.get_blueprint_library()
        chosen_vehicle_model = blueprint_library.filter('vehicle.tesla.model3')[0]
        chosen_vehicle_model.set_attribute('role_name', 'my_car')

        # # Spawning vehicle
        print("Attempting to spawn vehicle ...")
        vehicle = world.try_spawn_actor(chosen_vehicle_model,startPoint)
        print(f"Vehicle {chosen_vehicle_model.id} has spawn in {map.name}")

        # Setting up camera so it is behind vehicle
        intialize_specator(world,vehicle)

        # Create way points 
        routePlanner = GlobalRoutePlanner(map,sampleResolutoin)
        waypoints = create_way_points(world,routePlanner,startPoint,endPoint)
        draw_way_points(world, waypoints)

        # # Main Loop
        # initialTime = time.time()
        # while (time.time() - initialTime) < simulationTime:
        #     # Move vehicle 
        #     control = carla.VehicleControl(throttle=0.8, steer=0.0, brake=0.0) 
        #     vehicle.apply_control(control)

        #     # Updates camera to stay near vehicle 
        #     intialize_specator(world,vehicle)

        # # Stop vehicle 
        # control = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0) 
        # vehicle.apply_control(control)   
        
    except:
        print("Did not connect to Carla server")

if __name__ == '__main__':
    main()