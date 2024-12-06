import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt
import os
import pandas as pd

os.chdir(r'gear_ratio_results')

filename = '3.30769.pkl'
with open(filename, 'rb') as file:
    stuff = pkl.load(file)  # Load the figure directly
plt.close('all')

output = stuff['output']

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title('fuck off ~f(velocity, soc)  ~ type shit ~')
ax.set_xlabel('fuck off [m/s]')
ax.set_ylabel('fuck off [%]')
ax.set_zlabel('fuck off [A]')
torques = output['topology']['torque_requests']['values']
# torques = [output['topology']['torque_requests']['values'][1]]  # just look at the postitive torque curve for now.
socs = output['topology']['socs']['values']
velocities = output['topology']['velocities']['values']
for torque_req in torques:
    data = output['data'][torque_req]
    speed_grid, soc_grid = np.meshgrid(velocities, socs)
    torque_grid = np.array([[float(data[soc][velocity]['differential_torque']) for velocity in velocities] for soc in socs])
    if np.average(torque_grid) < 0:
        cmap = 'viridis_r'
    else:
        cmap = 'viridis'
    surf1 = ax.plot_surface(speed_grid, soc_grid, torque_grid, cmap=cmap)

fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_title('fuck off ~f(velocity, soc)  ~ type shit ~')
ax.set_xlabel('fuck off [m/s]')
ax.set_ylabel('fuck off [%]')
ax.set_zlabel('fuck off [Nm]')
for torque_req in torques:
    data = output['data'][torque_req]
    speed_grid, soc_grid = np.meshgrid(velocities, socs)
    power_grid = np.array([[float(data[soc][velocity]['motor_power']) for velocity in velocities] for soc in socs])
    if np.average(torque_grid) < 0:
        cmap = 'viridis_r'
    else:
        cmap = 'viridis'
    surf = ax.plot_surface(speed_grid, soc_grid, power_grid, cmap=cmap)

plt.show()


# export to excel
# data = output['data'][-220]
# speed_grid, soc_grid = np.meshgrid(velocities, socs)
# torque_grid = np.array([[data[soc][velocity]['differential_torque'] for velocity in velocities] for soc in socs])
# df = pd.DataFrame(torque_grid, index=soc_grid[:, 0], columns=speed_grid[0, :])
# df.index.name = 'soc [%]'
# df.columns.name = 'velocity [m/s]'
# df.to_excel('regen_torque~f(soc, veloctiy).xlsx')

print('siqmukmau')