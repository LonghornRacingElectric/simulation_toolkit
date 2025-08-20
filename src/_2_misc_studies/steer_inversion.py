from _4_custom_libraries.simulation import Simulation
from copy import deepcopy

import numpy as np

veh = Simulation(model_path="./_1_model_inputs/Nightwatch.yml", use_mode="kin")

mesh = 10
steer_sweep = np.linspace(-120, 120, mesh)
jounce_sweep = np.linspace(-5, 5, mesh) * 0.0254

steer_lst = []
jounce_lst = []
FL_mech_trail = []

counter = 0
for jounce in jounce_sweep:
    for hwa in steer_sweep:
        print(f"Progress: {round(counter / mesh**2 * 100, 2)}%", end="\r")
        veh.sus_copy = deepcopy(veh.sus)
        veh.sus_copy.heave(heave=jounce)
        veh.sus_copy.steer(hwa=hwa)

        steer_lst.append(hwa)
        jounce_lst.append(jounce)

        FL_mech_trail.append(veh.sus_copy.state["FL_mech_trail"] / 0.0254)
        counter += 1

steer_mesh, jounce_mesh = np.meshgrid(steer_sweep, jounce_sweep)
FL_mech_trail_mesh = np.reshape(np.array(FL_mech_trail), (mesh, mesh))

# jounce_mesh /= 0.0254

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Create figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot surface
surf = ax.plot_surface(steer_mesh, jounce_mesh, FL_mech_trail_mesh, cmap='viridis', edgecolor='none')
# sc = ax.scatter(steer_lst, jounce_lst, FL_mech_trail, c=FL_mech_trail, cmap='plasma', s=40)

# Add color bar and labels
ax.set_title("FL Mech Trail")
ax.set_xlabel("hwa (deg)")
ax.set_ylabel("jounce (in)")
ax.set_zlabel("mech_trail (in)")

plt.show()