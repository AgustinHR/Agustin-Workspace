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
#       - length() - returns magnitude of velocity vector (m/s)
#

################################# Packages ######################################
import sys
import carla 
import time
import math
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib.ticker import MultipleLocator

# Replace with where you have PythonAPI/carla located in your computer
sys.path.insert(1,'C:\\Users\\ejahi\\carla\\PythonAPI\\carla')

from agents.navigation.global_route_planner import GlobalRoutePlanner


class PIDControl:
    def __init__(self, Kp, Ki, Kd, ff):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.accumulatedError = 0.0
        self.previousError = 0.0
        self.previousTime = 0.0

    def controller(self, vref, v):
        currentTime = time.perf_counter()
        dt = currentTime - self.previousTime
        
        error = vref - v
        self.accumulatedError += error * dt
    
        porportionalTerm = self.Kp * error
        integralTerm = self.Ki * self.accumulatedError
        derivativeTerm = self.Kd * ( (error - self.previousError) / dt )

        self.previousError = error 

        u =  porportionalTerm + integralTerm + derivativeTerm + ff

        self.previousTime = currentTime

        if u >= 1.0:
            return min(u,1.0)
        elif u < 0.0:
            return 0.0
        
        return u

class Stanley:
    def __init__(self):
        self.wpi = 0

    def closest_path_point(self, F, xy_path):
        N = len(xy_path)
        idx = min(self.wpi, N - 2)
        wp1 = xy_path[idx]
        wp2 = xy_path[idx + 1]
        v = wp2 - wp1
        v_mag = np.linalg.norm(v)
        v_uv = v / v_mag

        s = (F-wp1) @ v_uv

        ######## REMOVE IF NOT NEEDED ###########
        # s = np.clip(s, 0.0, v_mag)
        ########################################
        
        if abs(s) >= v_mag and self.wpi < N-2:
            self.wpi += 1
                
        closestPoint = wp1  + v_uv * s
        crossTrackError = F - closestPoint

        ############ REMOVE AFTER ##################
        print(" ")
        print("*********************************************************************************")
        print("-------------------------------------------")
        print("Closest Path Point Status")
        print("-------------------------------------------")
        print(f"Current Location: {F}")
        print(f"WP1[{idx}]: {wp1} WP2[{idx+1}]: {wp2}")
        print(f"WP2-WP1: {v}")
        print(f"WP2-WP1 Magnitude: {v_mag}")
        print(f"Unit Vector of WP2-WP1: {v_uv}")
        print(f"Path Heading: { np.arctan2(v_uv[1], v_uv[0])}")
        print(f"Closest Point: {closestPoint}")
        print("-------------------------------------------")
        #############################################

        return crossTrackError, np.arctan2(v_uv[1], v_uv[0])
    
    def controller(self,crossTrackError, pathHeading, vehicleHeading, speed, k,k1):
        headingTerm = pathHeading - vehicleHeading

        if headingTerm > np.pi:
            headingTerm -= 2 * np.pi
        if headingTerm < -np.pi:
            headingTerm += 2 * np.pi

        crossTrackErrorMagnitude = np.linalg.norm(crossTrackError)

        if headingTerm > 0:
            crossTrackErrorMagnitude = abs(crossTrackErrorMagnitude)
        else:
            crossTrackErrorMagnitude = -abs(crossTrackErrorMagnitude)

        crossTrackTerm = np.arctan2(k*crossTrackErrorMagnitude,k1 + speed)


        steer = headingTerm + crossTrackTerm
        steer = np.clip(steer,-1.0,1.0)

        ############ REMOVE AFTER ##################
        print(" ")
        print("-------------------------------------------")
        print("Controller Status")
        print("-------------------------------------------")
        print(f"Path Heading: {pathHeading} Vehicle Heading: {vehicleHeading}")
        print(f"Heading Term: {headingTerm}")
        print(f"Cross Track Error: {crossTrackError}")
        print(f"Cross Track Error Magnitude: {crossTrackErrorMagnitude}")
        print(f"Cross Track Term: {crossTrackTerm}")
        print(f"Steer Command: {steer}")
        print("-------------------------------------------")
        print("*********************************************************************************")
        #############################################

        return steer

class PurePursuit:
    def __init__(self):
        self.last_index = 0


    def closest_path_point(self, F, xy_path):
        N = len(xy_path)
        idx = min(self.wpi, N - 2)
        wp1 = xy_path[idx]
        wp2 = xy_path[idx + 1]
        v = wp2 - wp1
        v_mag = np.linalg.norm(v)
        v_uv = v / v_mag

        s = (F-wp1) @ v_uv

        ######## REMOVE IF NOT NEEDED ###########
        # s = np.clip(s, 0.0, v_mag)
        ########################################
        
        if abs(s) >= v_mag and self.wpi < N-2:
            self.wpi += 1
                
        closestPoint = wp1  + v_uv * s
        crossTrackError = F - closestPoint

        ############ REMOVE AFTER ##################
        print(" ")
        print("*********************************************************************************")
        print("-------------------------------------------")
        print("Closest Path Point Status")
        print("-------------------------------------------")
        print(f"Current Location: {F}")
        print(f"WP1[{idx}]: {wp1} WP2[{idx+1}]: {wp2}")
        print(f"WP2-WP1: {v}")
        print(f"WP2-WP1 Magnitude: {v_mag}")
        print(f"Unit Vector of WP2-WP1: {v_uv}")
        print(f"Path Heading: { np.arctan2(v_uv[1], v_uv[0])}")
        print(f"Closest Point: {closestPoint}")
        print("-------------------------------------------")
        #############################################

        return crossTrackError, np.arctan2(v_uv[1], v_uv[0])

    def goal(self, vehicle, path, Ld):
        N = len(path)
        i = self.last_index
        physics = vehicle.get_physics_control()
        rearX = ((physics.wheels[2].position.x / 100.0) + (physics.wheels[3].position.x / 100.0)) / 2.0
        rearY = ((physics.wheels[2].position.y / 100.0) + (physics.wheels[3].position.y / 100.0)) / 2.0
        
        rearPosition = np.array([rearX, rearY])

        while i < N - 1:
            if np.linalg.norm(path[i] - rearPosition) >= Ld:
                break
            i += 1
        
        self.last_index = i 

        return path[i].copy(), rearPosition

    def controller(self, vehicle, xy_path, vehicleHeading, currentVelocity, L, Ld_min, Kv):
        Ld = max(Ld_min, Kv * currentVelocity)
        target,vehiclePosition = self.goal(vehicle, xy_path, Ld) # [x,y]
        x = target[0] - vehiclePosition[0]
        y = target[1] - vehiclePosition[1]
        alpha = np.arctan2(y,x) - vehicleHeading
        delta = np.arctan2(2*L*np.sin(alpha), Ld)
        return delta

##################### Provided by CARLA Documenation ###########################
def visualize_spawn_points_on_map(world, spawnPoints):
    for i, spawn_point in enumerate(spawnPoints):
        # Draw in the spectator window the spawn point index
        world.debug.draw_string(spawn_point.location, str(i), life_time=100)
        # We can also draw an arrow to see the orientation of the spawn point
        # (i.e. which way the vehicle will be facing when spawned)
        world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=100)

################################# Custom #######################################

def intialize_specator(world,vehicle):
    try:
        spectator = world.get_spectator()
        vehicle_current_location = vehicle.get_transform()
        adjusting_position = vehicle_current_location.location + carla.Location(y=7,z=2) # Adjust if camera position not over car
        adjusting_oreintation = carla.Rotation(0,-90,0) # Adjust if camera orientation is wrong [pitch,yaw,row]
        spectator.set_transform(carla.Transform(adjusting_position, adjusting_oreintation))
    except:
        print("Error: Something went wrong when trying to initialize spectator")

def cameraPosition(world, vehicle):
    spectator = world.get_spectator()
    vehicleCurrentLocaton = vehicle.get_transform()
    rectangularCord = vehicleCurrentLocaton.location + carla.Location(z=1.8)
    angularCord = vehicleCurrentLocaton.rotation
    spectator.set_transform(carla.Transform(rectangularCord, angularCord))

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


######### Original Gains ##########
# Kp = 1.2
# Ki = 0.03
# Kd = 0.01
# ff = 0.0
# k = 0.4
# k1 = 0.02
####################################

simulationTime = 25.0
startPoint = None
endPoint = None
sampleResolutoin = 0.02 # SEMI-STABLE FOR STanley0.5

controllerType = 'Pure Pursuit'

# PID paremters 
vref= 10.0
Kp = 0.335 # 1.2 # LEAVE
Ki = 0.0555 # LEAVE 
Kd = 0.0012 # LEAVE
ff = 0.0

# Stanely gain
k = 2.2 # 2.2 works 
k1 = 0.01 # 0.75

# Pure Pursuit 
L =  3.00464 # Approx. Wheel Base, BEFORE 1.55
Ld_min = 1.0 # Look Ahead 1.0
Kv = 0.8 # Adjustable gain 0.4 works

def main():

    client = None
    world = None
    vehicle = None 
    map = None
    waypoints = []
    xy_waypoints = []
    velocityAxis = []
    errorAxis = []
    crossTrackErrorData = []
    timeAxis = []
    carPosition = []

    try:
        # Lets connect to the Carla server
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.00)
        print("Connecting to Carla server")

        # Create world 
        world = client.get_world()
        client.load_world('Town02')
        map = world.get_map()
        print(f"Connecting to world {map.name}")

        # # Spawn points on the current map
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points found")
            return 
        startPoint = spawn_points[42] #50
        endPoint = spawn_points[80] # 100

        # # Choosing Vehicle
        blueprint_library = world.get_blueprint_library()
        chosen_vehicle_model = blueprint_library.filter('vehicle.tesla.model3')[0]
        chosen_vehicle_model.set_attribute('role_name', 'my_car')

        # # Spawning vehicle
        print("Attempting to spawn vehicle ...")
        vehicle = world.try_spawn_actor(chosen_vehicle_model,startPoint)
        print(type(vehicle))
        print(f"Vehicle {chosen_vehicle_model.id} has spawn in {map.name}")

        ######## TESTING (REMOVE IF IT DOES NOT WORK) #############
        # print(f"Type: {type(startPoint)}, Start Point: {startPoint.location.x}")
        # startPoint = carla.Location(startPoint.location.x + 100.0, startPoint.location.y, startPoint.location.z)
        # startPoint = carla.Transform(startPoint)
        # test_vehicle_transform = vehicle.get_transform()
        # new_location = startPoint.location + carla.Location(x=-2.0)
        # print(new_location)
        # new_transform = carla.Transform(new_location, startPoint.rotation)
        # vehicle.set_transform(new_transform)
        ###########################################################

        # Setting up camera so it is behind vehicle
        intialize_specator(world,vehicle)

        # Create way points 
        routePlanner = GlobalRoutePlanner(map,sampleResolutoin)
        waypoints = create_way_points(world,routePlanner,startPoint,endPoint)
        draw_way_points(world, waypoints)
        previousPointX = 0
        previousPointY = 0
        for w in waypoints:
            if ((w.x != previousPointX) & (w.y != previousPointY)):
                xy_waypoints.append([w.x,w.y])
                previousPointX = w.x
                previousPointY = w.y
        xy_waypoints = np.array(xy_waypoints)

        # Controlling vehicle 
        vehicleControl = carla.VehicleControl()

        # PID Controller
        pid = PIDControl(Kp,Ki,Kd,ff)
        pid.previousTime = time.perf_counter()

        # Stanley 
        stanley = Stanley()

        # Pure Pursuit
        purePursuit = PurePursuit()

        # Main Loop
        initialTime = time.time()
        
        while (time.time() - initialTime) < simulationTime:
            # Will be use for time-plot
            t = time.time() - initialTime

            # Current velocity 
            currentVelocity = vehicle.get_velocity().length()
            u = pid.controller(vref,currentVelocity)
            vehicleControl.throttle = u
            vehicle.apply_control(vehicleControl)

            # Position 
            vehicle_transform = vehicle.get_transform()
            vehicleHeading = math.radians(vehicle_transform.rotation.yaw)

            #------------------ Car Position (Center Of Wheel Base)---------------------------------------------------
            # currentLocation = np.array([vehicle.get_location().x, vehicle.get_location().y])
            # crossTrackError, pathHeading = stanley.closest_path_point(currentLocation, xy_waypoints)
            #---------------------------------------------------------------------------------------------------------

            #------------------ Car Position (Using get_physics_control() For Front Axle) (TESTING) ------------------
            # physics = vehicle.get_physics_control()
            # for i, wheels in enumerate(physics.wheels):
            #     print(f"i:{i}, wheel: {wheels.position/100.0}")
            # frontX = ((physics.wheels[0].position.x / 100.0) + (physics.wheels[1].position.x / 100.0)) / 2.0
            # frontY = ((physics.wheels[0].position.y / 100.0) + (physics.wheels[1].position.y / 100.0)) / 2.0
            # frontPosition = np.array([frontX, frontY])
            # crossTrackError, pathHeading = stanley.closest_path_point(frontPosition, xy_waypoints)
            #---------------------------------------------------------------------------------------------------------

            #---------- Car Position (Center of Vehcile Slightly Offset Towards The Front Axle) (TESTING) ------------
            x = vehicle_transform.location.x
            y = vehicle_transform.location.y

            print(f"Vehicle Location X: {x}")
            print(f"Vehicle Location Y: {y}")

            Lf = L / 2.0   # distance from vehicle center to front axle

            frontPositionTEST = np.array([
                x + Lf * math.cos(vehicleHeading),
                y + Lf * math.sin(vehicleHeading)
            ])

            # print(f"Front Position Using Physics: {frontPosition}")
            # print(f"Front Position Using World Cord: {frontPositionTEST}")

            crossTrackError, pathHeading = stanley.closest_path_point(
                frontPositionTEST,
                xy_waypoints
            )
            #-----------------------------------------------------------------------------------------------------------

            if controllerType == 'Stanley':
                # Stanely
                print('CONTROL TYPE: Stanley')
                steering = stanley.controller(crossTrackError,pathHeading,vehicleHeading,currentVelocity,k,k1)
                vehicleControl.steer = steering

            if controllerType == 'Pure Pursuit':
                # Pure Pursuit 
                print('CONTROL TYPE: Pure Pursuit')
                test = purePursuit.controller(vehicle,xy_waypoints,vehicleHeading,currentVelocity,L,Ld_min,Kv)
                vehicleControl.steer = test

            # Updates camera to stay near vehicle 
            # cameraPosition(world,vehicle)

            # Update time-plot
            velocityAxis.append(currentVelocity)
            errorAxis.append(pid.previousError)
            crossTrackErrorData.append(np.linalg.norm(crossTrackError))
            timeAxis.append(t)
            carPosition.append([vehicle.get_location().x, vehicle.get_location().y])

        # Stop vehicle 
        vehicleControl = carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0) 
        vehicle.apply_control(vehicleControl)  

        # Appending to plot 
        velocityAxis = np.array(velocityAxis)
        errorAxis = np.array(errorAxis)
        crossTrackErrorData = np.array(crossTrackErrorData)
        timeAxis = np.array(timeAxis)
        carPosition = np.array(carPosition)

        # Plot Settings
        plt.figure(1)
        plt.subplot(2,1,1)
        plt.xlabel("Time (s)")
        plt.ylabel("Vehicle Speed (m/s)")
        plt.title(f"PID Controller Results [Vref: {vref} Kp:{Kp}, Ki:{Ki}, Kd:{Kd}]")
        plt.axhline(y=vref,color='gray',linestyle='-')
        plt.minorticks_on()
        plt.grid(True, which="both", linewidth = 0.2)
        plt.plot(timeAxis, velocityAxis, linewidth="2")

        plt.subplot(2,1,2)
        plt.xlabel("Time (s)")
        plt.ylabel("Error")
        plt.title("PID Error")
        plt.minorticks_on()
        plt.grid(True, which="both", linewidth = 0.2)
        plt.plot(timeAxis,errorAxis, linewidth="2")

        # plt.subplot(2,2,3)
        # plt.xlabel("Time (s)")
        # plt.ylabel("Cross Track Error")
        # plt.title(f"{controllerType}: Cross Track Error")
        # plt.minorticks_on()
        # plt.grid(True, which="both", linewidth = 0.2)
        # plt.plot(timeAxis,crossTrackErrorData, linewidth="2")
       
        plt.figure(4)
        plt.subplot(1,1,1)
        plt.scatter(xy_waypoints[0,0], xy_waypoints[0,1], label="Starting Node",color="green",s=70)
        plt.plot(xy_waypoints[:,0],xy_waypoints[:,1], linewidth="2", label="Planned Trajectory")
        plt.plot(carPosition[:,0], carPosition[:,1], linewidth = "2", label="Actual Vehicle Trajectory")

        if controllerType == "Pure Pursuit":
            plt.title(f"Planned Trajectory For {controllerType} | Kv:{Kv}")
        else:
            plt.title(f"Planned Trajectory For {controllerType} | K:{k}")

        plt.xlabel("X-Axis")
        plt.ylabel("Y-Axis")
        plt.minorticks_on()
        plt.grid(True, which="both", linewidth = 0.2)
        plt.legend()
        plt.show()
    except:
        print("Did not connect to Carla server")

if __name__ == '__main__':
    main()