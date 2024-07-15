import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Parameters
N = 5  
master_index = 2  
positions = np.array([[0, 0], [20, 20], [40, 40], [60, 60], [80, 80]])  
m = 1.8 

tspan = (0, 200) 

# Initial conditions: [x1, y1, V_x1, V_y1, x2, y2, V_x2, V_y2, ..., xN, yN, V_xN, V_yN]
initial_conditions = np.hstack([positions, np.zeros((N, 2))]).flatten()

# Define the system of ODEs
def odesystem(t, state, N, m, master_index):
    d_coeff = 0.1
    Kx = 1
    Ky = 1
    K1 = 2
    K2 = 2
    
    dstate_dt = np.zeros(4 * N)
    for i in range(N):
        if i == master_index:
            continue  
        
        x_i = state[i * 4 + 0]
        y_i = state[i * 4 + 1]
        V_x_i = state[i * 4 + 2]
        V_y_i = state[i * 4 + 3]
        
        F_x = 0
        F_y = 0
        
        for j in range(N):
            if i != j:
                x_j = state[j * 4 + 0]
                y_j = state[j * 4 + 1]
                
                r = np.sqrt((x_i - x_j) ** 2 + (y_i - y_j) ** 2)
                theta = np.arctan2(y_j - y_i, x_j - x_i)
                
                F = 7.2 * ((10 / r) ** 4 - (10 / r) ** 2)
                
                F_x += Kx * F * np.cos(theta)
                F_y += Ky * F * np.sin(theta)
        
        dx_dt = V_x_i
        dy_dt = V_y_i
        dV_x_dt = K1 * (F_x - V_x_i)
        dV_y_dt = K2 * (F_y - V_y_i)
        
        dstate_dt[i * 4 + 0] = dx_dt
        dstate_dt[i * 4 + 1] = dy_dt
        dstate_dt[i * 4 + 2] = dV_x_dt
        dstate_dt[i * 4 + 3] = dV_y_dt
    
    return dstate_dt

# ODEs
sol = solve_ivp(odesystem, tspan, initial_conditions, args=(N, m, master_index), dense_output=True)


t = sol.t
result = sol.y.T
x_values = result[:, 0::4]
y_values = result[:, 1::4]

# Plot the motion of the drones
fig, ax = plt.subplots()
colors = plt.cm.viridis(np.linspace(0, 1, N))
lines = []
paths = []

for i in range(N):
    line, = ax.plot(positions[i, 0], positions[i, 1], 'o', markersize=5, color=colors[i], label=f'Drone {i}')
    path, = ax.plot(x_values[:, i], y_values[:, i], '-', color=colors[i], label=f'Path {i}')
    lines.append(line)
    paths.append(path)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('Motion of the Drones')
ax.legend()
ax.grid(True)
ax.axis('equal')

# Animate the motion
def update_plot(frame):
    for i in range(N):
        if i == master_index:
            continue  # Skip updating the master drone's position
        lines[i].set_data(x_values[frame, i], y_values[frame, i])
        paths[i].set_data(x_values[:frame + 1, i], y_values[:frame + 1, i])
    return lines + paths

from matplotlib.animation import FuncAnimation

ani = FuncAnimation(fig, update_plot, frames=len(t), interval=50, blit=True)
plt.show()
