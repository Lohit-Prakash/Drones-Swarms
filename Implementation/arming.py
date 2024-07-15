from dronekit import connect, VehicleMode
import time





target_altitude = 10

leader_udp = 'udp:127.0.0.1:14552'


leader = connect(leader_udp, wait_ready=True)


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

    print("Armed")

    # print("Taking off!")
    # vehicle.simple_takeoff(target_altitude)

    # while True:
    #     print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    #     if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
    #         print("Reached target altitude")
    #         break
    #     time.sleep(1)



# Arm and takeoff both drones
arm_and_takeoff(leader, target_altitude)
