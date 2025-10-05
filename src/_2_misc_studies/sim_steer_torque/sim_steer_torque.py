from src._3_custom_libraries.fmu_simulator import FMUSimulator

fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/sim_steer_torque/Kinematics_me.fmu")

print(fmu_sim.get_variables())

import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.gca()

cg_heights = [5*0.0254, 7.5*0.0254, 10*0.0254, 12.5*0.0254]

ax.set_title("CG Height vs Normalized Pinion Torque")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Normalized Pinion Torque")
plot_entries = []
for cg_height in cg_heights:
    start_values = {'CG_height': cg_height}
    sim_result = fmu_sim.simulate(start_time=0,
                                stop_time=2,
                                start_values=start_values)

    plot_entries.append((sim_result['time'], sim_result['pinion_torque'], f'z = {round(cg_height, 2)} m'))

normalizing_torque = max([max([abs(z) for z in y]) for y in [x[1] for x in plot_entries]]) # Don't try to understand it. Just trust it.

for time, sim_result, label in plot_entries:
    ax.plot(time, [x / normalizing_torque for x in sim_result], label=label)

ax.legend()
plt.show()
