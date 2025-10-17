from src._3_custom_libraries.fmu_simulator import FMUSimulator
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2_RL.fmu")
# fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2.fmu")
print(fmu_sim.get_variables())

time = np.linspace(0, 2, 8001)

def create_c2_sweep(time_array, neg_peak, pos_peak):
    output = np.zeros_like(time_array)
    t_total = time_array[-1]
    t_phase1_end = t_total * 0.25
    t_phase2_end = t_total * 0.75
    t_phase3_end = t_total
    def smooth_step(t_norm):
        return t_norm**3 * (10 + t_norm * (-15 + t_norm * 6))
    mask1 = time_array <= t_phase1_end
    t_norm1 = time_array[mask1] / t_phase1_end
    output[mask1] = neg_peak * smooth_step(t_norm1)
    mask2 = (time_array > t_phase1_end) & (time_array <= t_phase2_end)
    t_norm2 = (time_array[mask2] - t_phase1_end) / (t_phase2_end - t_phase1_end)
    output[mask2] = neg_peak + (pos_peak - neg_peak) * smooth_step(t_norm2)
    mask3 = time_array > t_phase2_end
    t_norm3 = (time_array[mask3] - t_phase2_end) / (t_phase3_end - t_phase2_end)
    output[mask3] = pos_peak * (1 - smooth_step(t_norm3))
    return output

jounce_travel = 2 * 0.0254
rack_travel = 0.0254
jounce_vals = create_c2_sweep(time, -jounce_travel, jounce_travel)
rack_vals = create_c2_sweep(time, -rack_travel, rack_travel) * 0.5

plt.figure()
plt.plot(time, rack_vals)
plt.show()
plt.figure()
plt.plot(time, jounce_vals)
plt.show()

input_data = np.zeros(time.shape[0],
    dtype=[('time', np.float64),
           ('jounce', np.float64),
           ('rack_input', np.float64)]
)
input_data['time'] = time
input_data['jounce'] = jounce_vals
input_data['rack_input'] = rack_vals

sim_result = fmu_sim.simulate(
    fmi_type="ModelExchange",
    start_time=0,
    stop_time=2,
    step_size=0.00004,
    input_data=input_data,
    output_vars=['camber', 'toe'],
    debug_logging=False,
    start_values={'jounce': 0.0, 'rack_input': 0.0},
    relative_tolerance=1e-5
)

print("Simulation finished")
print("Result keys:", sim_result.dtype.names)

time_res = sim_result['time']
camber = sim_result['camber']
toe = sim_result['toe']

rack_interp = np.interp(time_res, input_data['time'], input_data['rack_input'])
jounce_interp = np.interp(time_res, input_data['time'], input_data['jounce'])

mask = (time_res >= 0.5) & (time_res <= 1.5)
time_res = time_res[mask]
camber = camber[mask]
toe = toe[mask]
rack_interp = rack_interp[mask]
jounce_interp = jounce_interp[mask]

plt.figure()
plt.plot(rack_interp * 1000, np.degrees(toe), label='Toe')
plt.xlabel('Rack Input [mm]')
plt.ylabel('Toe [deg]')
plt.title('Toe vs Rack Input (FMU)')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(jounce_interp * 1000, np.degrees(camber), label='Camber')
plt.xlabel('Jounce [mm]')
plt.ylabel('Camber [deg]')
plt.title('Camber vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()

def continuous_zero_cross_section(x, y, z, n_points=300):
    z_f = interp1d(x, z, kind='linear', fill_value='extrapolate')
    y_f = interp1d(x, y, kind='linear', fill_value='extrapolate')
    x_new = np.linspace(np.min(x), np.max(x), n_points)
    z_new = z_f(x_new)
    y_new = y_f(x_new)
    z_absmin_idx = np.argmin(np.abs(z_new))
    idx_window = max(0, z_absmin_idx - 10), min(len(x_new), z_absmin_idx + 10)
    x_window = x_new[idx_window[0]:idx_window[1]]
    y_window = y_new[idx_window[0]:idx_window[1]]
    return x_window, y_window

rack_at_zero_jounce, camber_at_zero_jounce = continuous_zero_cross_section(
    x=rack_interp, y=camber, z=jounce_interp
)

plt.figure()
plt.plot(rack_at_zero_jounce * 1000, np.degrees(camber_at_zero_jounce), '-o')
plt.xlabel('Rack Input [mm]')
plt.ylabel('Camber [deg]')
plt.title('Camber vs Rack Input (at Jounce ≈ 0)')
plt.grid(True)
plt.show()

jounce_at_zero_rack, toe_at_zero_rack = continuous_zero_cross_section(
    x=jounce_interp, y=toe, z=rack_interp
)

plt.figure()
plt.plot(jounce_at_zero_rack * 1000, np.degrees(toe_at_zero_rack), '-o')
plt.xlabel('Jounce [mm]')
plt.ylabel('Toe [deg]')
plt.title('Toe vs Jounce (at Rack ≈ 0)')
plt.grid(True)
plt.show()
