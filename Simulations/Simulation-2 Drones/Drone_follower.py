


import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Parameters
x1 = 0  # Leader point x-coordinate
y1 = 0  # Leader point y-coordinate
x2 = 20  # Initial follower point x-coordinate
y2 = 20  # Initial follower point y-coordinate
m = 1.8  # Mass of the follower

# Time span for the simulation
tspan = np.linspace(0, 200, 1000)  # Adjust as needed



# Initial conditions: [x2, y2, V_x, V_y]
initial_conditions = [x2, y2, 0, 0]

# Define the system of ODEs
def odesystem(state, t, x1, y1, m):
    x, y, V_x, V_y = state
    d_coeff = 0.1
    Kx = 1
    Ky = 1
    K1 = 2
    K2 = 2

    r = np.sqrt((x - x1)**2 + (y - y1)**2)
    theta = np.arctan2(y - y1, x - x1)
    
    F = 7.2 * ((10 / r)**4 - (10 / r)**2)
    Vxc = Kx * F * np.cos(theta)
    Vyc = Ky * F * np.sin(theta)
    
    dx_dt = V_x
    dy_dt = V_y
    dV_x_dt = K1 * (Vxc - V_x)
    dV_y_dt = K2 * (Vyc - V_y)
    
    return [dx_dt, dy_dt, dV_x_dt, dV_y_dt]

# Solve the system of ODEs
result = odeint(odesystem, initial_conditions, tspan, args=(x1, y1, m))

# Extract the results
x2_values = result[:, 0]
y2_values = result[:, 1]

print(f"X= {x2_values}")
print(f"Y= {y2_values}")

# Plot the motion of the follower point
plt.figure()
plt.plot(x1, y1, 'ro', markersize=10, label='Leader Point')
follower, = plt.plot(x2, y2, 'bo', markersize=5, label='Follower Point')
path, = plt.plot(x2_values[0], y2_values[0], 'b-', label='Follower Path')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Motion of the Follower Point')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Animate the motion
for k in range(len(tspan)):
    # Update the follower's position
    follower.set_xdata(x2_values[k])
    follower.set_ydata(y2_values[k])
    
    # Update the path
    path.set_xdata(x2_values[:k])
    path.set_ydata(y2_values[:k])
    
    # Pause for a short duration to create animation effect
    plt.pause(0.05)

plt.show()





