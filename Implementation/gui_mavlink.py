import tkinter as tk
from tkinter import messagebox
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import threading

# Connect to the vehicle via Mission Planner's UDP connection
print("Connecting to vehicle on: 127.0.0.1:14551")
vehicle = connect('127.0.0.1:14551', baud=115200, wait_ready=True)

# Available flight modes
FLIGHT_MODES = ["STABILIZE", "ALT_HOLD", "LOITER", "LAND", "RTL", "AUTO", "GUIDED", "POSHOLD"]

# Function to change the flight mode
def change_mode(mode):
    try:
        while vehicle.mode != VehicleMode(mode):
            vehicle.mode = VehicleMode(mode)
            time.sleep(1)
        messagebox.showinfo("Info", f"Mode changed to {mode}")
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Function to arm and take off
def arm_and_takeoff(target_altitude):
    try:
        print("Arming motors")
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True

        while not vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        vehicle.simple_takeoff(target_altitude)

        while True:
            print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
            if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
        messagebox.showinfo("Info", f"Reached target altitude of {target_altitude} meters")
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Function to land the vehicle
def land():
    try:
        print("Landing")
        vehicle.mode = VehicleMode("LAND")
        while vehicle.armed:
            print(" Waiting for disarming...")
            time.sleep(1)
        messagebox.showinfo("Info", "Landed and disarmed")
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Function to handle the takeoff button click
def handle_takeoff():
    altitude = entry_altitude.get()
    if altitude.isdigit():
        altitude = float(altitude)
        threading.Thread(target=arm_and_takeoff, args=(altitude,)).start()
    else:
        messagebox.showerror("Error", "Please enter a valid altitude")

# Function to handle the change mode button click
def handle_change_mode():
    mode = mode_var.get()
    threading.Thread(target=change_mode, args=(mode,)).start()

# Function to handle the land button click
def handle_land():
    threading.Thread(target=land).start()

# Function to handle moving to a specific location
def handle_go_to():
    lat = entry_latitude.get()
    lon = entry_longitude.get()
    alt = entry_target_altitude.get()
    if lat and lon and alt:
        try:
            lat = float(lat)
            lon = float(lon)
            alt = float(alt)
            location = LocationGlobalRelative(lat, lon, alt)
            vehicle.simple_goto(location)
            messagebox.showinfo("Info", "Moving to target location")
        except ValueError:
            messagebox.showerror("Error", "Please enter valid coordinates and altitude")
    else:
        messagebox.showerror("Error", "Please enter latitude, longitude, and altitude")

# Function to handle pitch, roll, and yaw
def handle_pitch_roll_yaw():
    try:
        pitch = float(entry_pitch.get())
        roll = float(entry_roll.get())
        yaw = float(entry_yaw.get())
        
        # Sending manual control commands (example; needs to be adjusted according to the actual vehicle control mechanism)
        vehicle.channels.overrides['1'] = pitch  # Aileron (Roll)
        vehicle.channels.overrides['2'] = roll   # Elevator (Pitch)
        vehicle.channels.overrides['4'] = yaw    # Rudder (Yaw)
        
        messagebox.showinfo("Info", f"Control input - Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")
    except ValueError:
        messagebox.showerror("Error", "Please enter valid pitch, roll, and yaw values")

# Close the vehicle connection on GUI exit
def on_closing():
    vehicle.close()
    root.destroy()

# Create the main window
root = tk.Tk()
root.title("Quadcopter Control")

# Create and place the widgets
tk.Label(root, text="Flight Mode:").grid(row=0, column=0, padx=10, pady=10)
mode_var = tk.StringVar(root)
mode_var.set(FLIGHT_MODES[0])  # Set the default mode
option_menu = tk.OptionMenu(root, mode_var, *FLIGHT_MODES)
option_menu.grid(row=0, column=1, padx=10, pady=10)

btn_change_mode = tk.Button(root, text="Change Mode", command=handle_change_mode)
btn_change_mode.grid(row=0, column=2, padx=10, pady=10)

tk.Label(root, text="Target Altitude (m):").grid(row=1, column=0, padx=10, pady=10)
entry_altitude = tk.Entry(root)
entry_altitude.grid(row=1, column=1, padx=10, pady=10)

btn_takeoff = tk.Button(root, text="Arm and Takeoff", command=handle_takeoff)
btn_takeoff.grid(row=1, column=2, padx=10, pady=10)

btn_land = tk.Button(root, text="Land", command=handle_land)
btn_land.grid(row=2, column=0, columnspan=3, pady=10)

tk.Label(root, text="Latitude:").grid(row=3, column=0, padx=10, pady=10)
entry_latitude = tk.Entry(root)
entry_latitude.grid(row=3, column=1, padx=10, pady=10)

tk.Label(root, text="Longitude:").grid(row=4, column=0, padx=10, pady=10)
entry_longitude = tk.Entry(root)
entry_longitude.grid(row=4, column=1, padx=10, pady=10)

tk.Label(root, text="Target Altitude (m):").grid(row=5, column=0, padx=10, pady=10)
entry_target_altitude = tk.Entry(root)
entry_target_altitude.grid(row=5, column=1, padx=10, pady=10)

btn_go_to = tk.Button(root, text="Go To Location", command=handle_go_to)
btn_go_to.grid(row=5, column=2, padx=10, pady=10)

tk.Label(root, text="Pitch:").grid(row=6, column=0, padx=10, pady=10)
entry_pitch = tk.Entry(root)
entry_pitch.grid(row=6, column=1, padx=10, pady=10)

tk.Label(root, text="Roll:").grid(row=7, column=0, padx=10, pady=10)
entry_roll = tk.Entry(root)
entry_roll.grid(row=7, column=1, padx=10, pady=10)

tk.Label(root, text="Yaw:").grid(row=8, column=0, padx=10, pady=10)
entry_yaw = tk.Entry(root)
entry_yaw.grid(row=8, column=1, padx=10, pady=10)

btn_pitch_roll_yaw = tk.Button(root, text="Set Pitch/Roll/Yaw", command=handle_pitch_roll_yaw)
btn_pitch_roll_yaw.grid(row=8, column=2, padx=10, pady=10)

# Set the window close protocol
root.protocol("WM_DELETE_WINDOW", on_closing)

# Start the Tkinter event loop
root.mainloop()
