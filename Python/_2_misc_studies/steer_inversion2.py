from _4_custom_libraries.simulation import Simulation
from copy import deepcopy

import numpy as np

veh = Simulation(model_path="./_1_model_inputs/Nightwatch.yml", use_mode="kin")

mesh = 10
steer_sweep = np.linspace(-120, 120, mesh)
FL_mech_trail = []

counter = 0
for hwa in steer_sweep:
    veh.sus_copy = deepcopy(veh.sus)
    veh.sus_copy.steer(hwa=hwa)

    FL_mech_trail.append(veh.sus_copy.state["FL_mech_trail"] / 0.0254)
    counter += 1

import matplotlib.pyplot as plt

# Create figure and 3D axis
fig = plt.figure()
ax = fig.gca()

# Plot surface
ax.plot(steer_sweep, np.array(FL_mech_trail) / 0.0254)

plt.show()