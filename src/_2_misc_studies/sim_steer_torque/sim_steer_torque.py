from src._3_custom_libraries.fmu_simulator import FMUSimulator

fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/sim_steer_torque/Kinematics_me.fmu")

print(fmu_sim.get_variables())

import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.gca()

cg_heights = [5*0.0254, 7.5*0.0254, 10*0.0254, 12.5*0.0254]

for cg_height in cg_heights:
    start_values = {'CG_height': cg_height}
    sim_result = fmu_sim.simulate(start_time=0,
                                stop_time=2,
                                start_values=start_values)

    ax.plot(sim_result['time'], sim_result['pinion_torque'])

plt.show()
