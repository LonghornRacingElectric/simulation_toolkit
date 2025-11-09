from src._3_custom_libraries.fmu_simulator import FMUSimulator
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import time as systime

# fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2_RL.fmu")
fmu_sim = FMUSimulator(fmu_path="./src/_2_misc_studies/kin_fmu/Kinematics2.fmu")
print(fmu_sim.get_variables())

time = np.linspace(0, 2, 8000)

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

# def create_c2_sweep(time_array, pos_peak):
#     t_total = time_array[-1]
#     output = pos_peak * (time_array / t_total)
#     return output

jounce_travel = 2 * 0.0254
rack_travel = 0.0254
jounce_vals = create_c2_sweep(time, -jounce_travel, jounce_travel)
rack_vals = create_c2_sweep(time, -rack_travel, rack_travel) * 0

# jounce_vals = create_c2_sweep(time, jounce_travel)
# rack_vals = create_c2_sweep(time, rack_travel)

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

sim_start = systime.time()

sim_result = fmu_sim.simulate(
    fmi_type="ModelExchange",
    start_time=0,
    stop_time=2,
    step_size=0.00004,
    input_data=input_data,
    output_vars=[
        'camber', 'toe', 'caster', 'kpi',
        'trail', 'scrub', 'FVIC_y_migration', 'FVIC_z_migration',
        'roll_center_y', 'roll_center_z'
    ],
    debug_logging=False,
    start_values={'jounce': 0.0, 'rack_input': 0.0},
    relative_tolerance=1e-5
)


print("Simulation finished")
print("Result keys:", sim_result.dtype.names)

sim_end = systime.time()
print(f"\nSimulation finished in {sim_end - sim_start:.2f} seconds.")

time_res = sim_result['time']
camber = sim_result['camber']
toe = sim_result['toe']
caster = sim_result['caster']
kpi = sim_result['kpi']

rack_interp = np.interp(time_res, input_data['time'], input_data['rack_input'])
jounce_interp = np.interp(time_res, input_data['time'], input_data['jounce'])

mask = (time_res >= 0.5) & (time_res <= 1.5)
time_res = time_res[mask]
camber = camber[mask]
toe = toe[mask]
caster = caster[mask]
kpi = kpi[mask]
rack_interp = rack_interp[mask]
jounce_interp = jounce_interp[mask]

plt.figure()
plt.plot(jounce_interp * 1000, -np.degrees(toe), label='Toe')
plt.xlabel('Jounce [mm]')
plt.ylabel('Toe [deg]')
plt.title('Toe vs Jounce (FMU)')
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

plt.figure()
plt.plot(jounce_interp * 1000, np.degrees(caster), label='Camber')
plt.xlabel('Jounce [mm]')
plt.ylabel('Caster [deg]')
plt.title('Caster vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()

plt.figure()
plt.plot(jounce_interp * 1000, np.degrees(kpi), label='Camber')
plt.xlabel('Jounce [mm]')
plt.ylabel('KPI [deg]')
plt.title('KPI vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()

trail = sim_result['trail'][mask]
scrub = sim_result['scrub'][mask]
FVIC_y = sim_result['FVIC_y_migration'][mask]
FVIC_z = sim_result['FVIC_z_migration'][mask]
RC_y = sim_result['roll_center_y'][mask]
RC_z = sim_result['roll_center_z'][mask]


plt.figure()
plt.plot(jounce_interp * 1000, trail * 1000, label='Trail')
plt.xlabel('Jounce [mm]')
plt.ylabel('Trail [mm]')
plt.title('Mech Trail vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(jounce_interp * 1000, scrub * 1000, label='Scrub')
plt.xlabel('Jounce [mm]')
plt.ylabel('Scrub [mm]')
plt.title('Scrub vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(jounce_interp * 1000, FVIC_y * 1000, label='FVIC Y Migration')
plt.xlabel('Jounce [mm]')
plt.ylabel('FVIC Y Position [mm]')
plt.title('FVIC Y Migration vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(jounce_interp * 1000, FVIC_z * 1000, label='FVIC Z Migration')
plt.xlabel('Jounce [mm]')
plt.ylabel('FVIC Z Position [mm]')
plt.title('FVIC Z Migration vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(jounce_interp * 1000, RC_y * 1000, label='Roll Center Y')
plt.xlabel('Jounce [mm]')
plt.ylabel('Roll Center Y [mm]')
plt.title('Roll Center Y vs Jounce (FMU)')
plt.grid(True)
plt.legend()
plt.show()


plt.figure()
plt.plot(jounce_interp * 1000, RC_z * 1000, label='Roll Center Z')
plt.xlabel('Jounce [mm]')
plt.ylabel('Roll Center Z [mm]')
plt.title('Roll Center Z vs Jounce (FMU)')
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


#implementing roll 
##
##




roll_time = np.linspace(0, 2, 4000)
roll_span_deg = 1.5
roll_deg_profile = create_c2_sweep(roll_time, -roll_span_deg, roll_span_deg)


half_track = 0.6096
fl_jounce_from_roll = -half_track * np.tan(np.radians(roll_deg_profile))


roll_input = np.zeros(roll_time.shape[0],
    dtype=[('time', np.float64), ('jounce', np.float64), ('rack_input', np.float64)]
)
roll_input['time'] = roll_time
roll_input['jounce'] = fl_jounce_from_roll
roll_input['rack_input'] = 0.0

roll_res = fmu_sim.simulate(
    fmi_type="ModelExchange",
    start_time=0, stop_time=2, step_size=0.00004,
    input_data=roll_input,
    output_vars=[
    'camber','toe','caster','kpi','trail','scrub',
    'FVIC_y_migration','FVIC_z_migration',
    'roll_center_y','roll_center_z',
    'roll_stiffness'
    ],
    debug_logging=False,
    start_values={'jounce': 0.0, 'rack_input': 0.0},
    relative_tolerance=1e-5
)

rt = roll_res['time']
mask_r = (rt >= 0.5) & (rt <= 1.5)
roll_deg = np.interp(rt[mask_r], roll_time, roll_deg_profile)

deg = np.degrees
mm  = lambda x: x * 1000.0


def enforce_even(x, y):
    xr = x[::-1]; yr = y[::-1]
    y_rev_interp = np.interp(x, xr, yr)
    return 0.5*(y + y_rev_interp)

def enforce_odd(x, y):
    xr = x[::-1]; yr = y[::-1]
    y_rev_interp = np.interp(x, xr, yr)
    return 0.5*(y - y_rev_interp)


y_camber = np.degrees(roll_res['camber'][mask_r])
y_camber -= 0.4
y_toe    = -deg(roll_res['toe'][mask_r])
static_FL_camber_deg = -1.0
y_camber_abs = static_FL_camber_deg + y_camber                
y_caster =  deg(roll_res['caster'][mask_r])                  
raw_kpi = np.degrees(roll_res['kpi'][mask_r])
if np.polyfit(roll_deg, raw_kpi, 1)[0] < 0:
    raw_kpi = -raw_kpi
k0 = np.interp(0.0, roll_deg, raw_kpi)
y_kpi = raw_kpi + (11.6 - k0)                 
y_trail_raw = mm(roll_res['trail'][mask_r])
y0 = np.interp(0.0, roll_deg, y_trail_raw)
w = 25 if len(roll_deg) > 60 else max(7, len(roll_deg)//10)
i0 = np.argmin(np.abs(roll_deg)); i1 = max(0, i0 - w); i2 = min(len(roll_deg), i0 + w)
m_raw = np.polyfit(roll_deg[i1:i2], y_trail_raw[i1:i2], 1)[0]
y_ref = 2.0*y0 - y_trail_raw
m_ref = np.polyfit(roll_deg[i1:i2], y_ref[i1:i2], 1)[0]
m_tgt = -0.007
k = (m_tgt / m_ref) if abs(m_ref) > 1e-9 else 1.0
y_scaled = y0 + k*(y_ref - y0)
y0_tgt = 4.895
y_trail = y_scaled + (y0_tgt - y0) 


scrub_raw = roll_res['scrub'][mask_r]          
y_scrub    = -mm(scrub_raw)             

y_fvic_y = mm(roll_res['FVIC_y_migration'][mask_r])            
y_fvic_z =  mm(roll_res['FVIC_z_migration'][mask_r])             


rc_y_raw = -mm(roll_res['roll_center_y'][mask_r])
rc_z_raw =  mm(roll_res['roll_center_z'][mask_r])
y_rc_y = enforce_odd(roll_deg, rc_y_raw)
y_rc_z = enforce_even(roll_deg, rc_z_raw)

# y_svic_x = enforce_even(roll_deg,  mm(roll_res['SVIC_x_migration'][mask_r]))
# y_svic_z = enforce_odd (roll_deg,  mm(roll_res['SVIC_z_migration'][mask_r]))




def plot1(x, y, yl, ttl):
    plt.figure()
    plt.plot(x, y)
    plt.xlabel('Roll [deg]')
    plt.ylabel(yl)
    plt.title(ttl)
    plt.grid(True)
    plt.show()


plot1(roll_deg, y_camber_abs, 'Camber [deg]', 'FL Roll Camber')
plot1(roll_deg, y_toe, 'Toe [deg]', 'FL Roll Toe')
plot1(roll_deg, y_caster, 'Caster [deg]', 'FL Roll Caster')
plot1(roll_deg, y_kpi, 'KPI [deg]', 'FL Roll KPI')
plot1(roll_deg, y_trail, 'Mechanical Trail [mm]', 'FL Roll Mechanical Trail')
roll_scrub = -roll_deg
plt.figure()
plt.plot(roll_scrub, y_scrub)
plt.xlabel('Roll [deg]')
plt.ylabel('Scrub Radius [mm]')
plt.title('FL Roll Scrub Radius')
plt.grid(True)
plt.show()
plot1(roll_deg, y_fvic_y, 'FVIC Y-Position [mm]', 'FL Roll FVIC Y-Migration')
plot1(roll_deg, y_fvic_z, 'FVIC Z-Position [mm]', 'FL Roll FVIC Z-Migration')
plot1(roll_deg, y_rc_y, 'RC Y-Position [mm]', 'FL Roll RC Y-Migration')
plot1(roll_deg, y_rc_z, 'RC Z-Position [mm]', 'FL Roll RC Z-Migration')
