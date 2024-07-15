from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import numpy as np
from pymavlink import mavutil
import math

# x1, y1 = -35.36614170168917, 149.16367849468412  # Leader coordinates
x1, y1 = -35.366815346227845, 149.1644080578529
m = 1.8  # Mass of the follower


initial_conditions = [-35.363276469051556, 149.16527354249652, 0, 0]  # Initial follower position and velocities

# Time span for the simulation
tspan = np.linspace(0, 200, 1000)  # Adjust as needed

dis = 150 # The dis to be maintained between the Leader and the follower 
LDuration = 20  # time - leader to reach x1 and y1
target_altitude = 10

leader_udp = 'udp:127.0.0.1:14552'
follower_udp = 'udp:127.0.0.1:14553'


def haversine(lat1, lon1, lat2, lon2):
    # Convert to Radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) #Central Angle
    r = 6371000  # Radius of Earth in meters
    return r * c





#defining force logic 
def logic(state, x1, y1):
    x, y = state

    Kx = 10
    Ky = 10

    # r = np.sqrt((x - x1)**2 + (y - y1)**2)
    r = haversine(x, y, x1, y1)
    theta = np.arctan2(y - y1, x - x1)
    print("Value or R : ", r)
    F = 7.2 * ((dis / r)**4 - (dis/ r)**2)
    Vxc = Kx * F * np.cos(theta)
    Vyc = Ky * F * np.sin(theta)
    print("Value or Force : ", F)
    return [Vxc , Vyc]

# Connect to the Leader Drone
leader = connect(leader_udp, wait_ready=True)

# Connect to the Follower Drone
follower = connect(follower_udp, wait_ready=True)

# Arm and takeoff function
def arm_and_takeoff(vehicle, target_altitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)



# Arm and takeoff both drones
arm_and_takeoff(leader, target_altitude)
arm_and_takeoff(follower, target_altitude)



# Leader is fixed at a single location
leader_location = LocationGlobalRelative(x1, y1, target_altitude)
leader.simple_goto(leader_location)
time.sleep(LDuration)
print("I guess Reached")

# Function to send velocity commands to the follower drone

def send_velocity_command(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame set to LOCAL_NED   Perivously - Set to MAV_FRAME_BODY_NED where axis is not locked 
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)    # yaw, yaw_rate (not used)
    vehicle.send_mavlink(msg)
    vehicle.flush()



# Continuously update the position and velocity of the follower drone
while True:

    follower_location = follower.location.global_relative_frame
    fx, fy = follower_location.lat, follower_location.lon



    Vx = follower.velocity[0]  # Current velocity in x-direction
    Vy = follower.velocity[1]  # Current velocity in y-direction
    Vx_cmd , Vy_cmd = logic([fx,fy],x1, y1)


    # V_net = np.sqrt(Vx_cmd**2 +Vy_cmd**2)
    # if V_net > 10 :
    #     Vx_cmd = 7.3
    #     Vy_cmd = 7.3


    print("Running")


    print(f"Current Position: lat={fx}, lon={fy}, Vx={Vx}, Vy={Vy}")
    print(f"Commanded Position: lat={fx}, lon={fy}, Vx_cmd={Vx_cmd}, Vy_cmd={Vy_cmd}")
    send_velocity_command(follower, round(Vx_cmd,1), round(Vy_cmd,1), 0)

    time.sleep(1)

# Close the vehicle objects
leader.close()
follower.close()
