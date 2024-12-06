import numpy as np
import pickle as pkl
import matplotlib.pyplot as plt
import os
import pandas as pd
from matplotlib.lines import Line2D

# os.chdir(r'C:\Users\simmons_\Documents\lhre git')
os.chdir(r'C:\Users\andre\Documents\coding')
orig_dir = os.getcwd()
print(os.getcwd())

os.chdir('simulation_toolkit/vehicle_model/powertrain_model/analysis/torque_speed_requirements')
target = pd.read_csv('torque_speed_req.csv')
os.chdir(orig_dir)
print(os.getcwd())

target_tor, target_vel = target['torque'], target['velocity']

os.chdir('simulation_toolkit/vehicle_model/powertrain_model/analysis/outputs/gear_ratio_results')
files = os.listdir()
files = np.flip(np.sort([file for file in files if '.pkl' in file]))
# files = ['3.15_results.pkl']

# colors = np.linspace(0,1, len(files))
# colors = [(color, 0 ,0) for color in colors]
colors = ['r', 'g', 'b', 'c', 'y', 'k', (1.0, 0.0, 0.7215), (0., 0.94117647, 0.99607843)]
loaded_files = []
for i, file in enumerate(files):
    filename = str(file)
    with open(filename, 'rb') as f:
        stuff = pkl.load(f)  # Load the figure directly7
    loaded_files = np.append(loaded_files, stuff)
plt.close('all')
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')
# ax.set_title('wheel torque ~f(velocity, soc)  ~ type shit ~')
ax.set_xlabel('velocity [m/s]')
ax.set_ylabel('soc [%]')
ax.set_zlabel('axle torque [Nm]')
legend_elements = []
for i, file in enumerate(loaded_files):
    output = file['output']
    socs = output['topology']['socs']['values']
    velocities = output['topology']['velocities']['values']
    data = output['data'][220]
    speed_grid, soc_grid = np.meshgrid(velocities, socs)
    torque_grid = np.array([[data[soc][velocity]['differential_torque'] for velocity in velocities] for soc in socs])
    gr = file['parameters_file']['gear_ratio']['value']
    ax.plot_wireframe(speed_grid, soc_grid, torque_grid, color=colors[i], alpha=1)
    legend_elements = np.append(legend_elements, Line2D([0], [0],
                                                        color=colors[i], lw=4, label='gear_ratio: ' + str(gr)))

socs = np.linspace(min(socs), max(socs), 2)
speed_grid, soc_grid = np.meshgrid(target_vel, socs)
torque_grid = np.array([target_tor for soc in socs])
surf = ax.plot_surface(speed_grid, soc_grid, torque_grid, color=(0.2, 0.2, 0.2), alpha=0.5)
legend_elements = list(np.append(legend_elements, Line2D([0], [0], color=(0.2, 0.2, 0.2), lw=4,
                                                        label='target')))
ax.legend(handles=legend_elements, loc='upper right')
plt.show()



print('siqmukmau')