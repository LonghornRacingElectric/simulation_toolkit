from src._3_custom_libraries.fmu_simulator import FMUSimulator
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import time as systime


class Kinematics_FMU:
    def __init__(self, fmu_path: str, sim_time: float = 2.0, step_size = 4e-5):
        self.time = None
        self.rt = None
        self.outputs = None
        self.roll_outputs = None
        self.fmu_path = fmu_path
        self.sim_time = sim_time
        self.step_size = step_size
        self.fmu_sim = FMUSimulator(fmu_path=fmu_path)
        self.roll_deg = None
        self.roll_deg_fmu = None
        
    def input_sweep(self, time_array, neg_peak, pos_peak):
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

    def run_simulation_jounce(self, jounce_travel = 2 * 0.0254, rack_travel = 0):
        time = np.linspace(0, 2, 8000)
        jounce_vals = self.input_sweep(time, -jounce_travel, jounce_travel)
        rack_vals = self.input_sweep(time, -rack_travel, rack_travel)
        
        self.input_data = np.zeros(time.shape[0],
        dtype=[('time', np.float64),
           ('jounce', np.float64),
           ('rack_input', np.float64)]
        )
        self.input_data['time'] = time
        self.input_data['jounce'] = jounce_vals
        self.input_data['rack_input'] = rack_vals 

        sim_start = systime.time()

        sim_result = self.fmu_sim.simulate(
        fmi_type="ModelExchange",
        start_time=0,
        stop_time=2,
        step_size=0.00004,
        input_data=self.input_data,
        output_vars=[
            'camber', 'toe', 'caster', 'kpi',
            'trail', 'scrub', 'FVIC_y_migration', 'FVIC_z_migration',
            'roll_center_y', 'roll_center_z'
        ],
        debug_logging=False,
        start_values={'jounce': 0.0, 'rack_input': 0.0},
        relative_tolerance=1e-5
        )

        sim_end = systime.time()
        
        self.time = sim_result['time']
        self.outputs = sim_result

    def run_simulation_roll(self, roll_travel = 1.5):
        if self.outputs is None:
            raise RuntimeError("You must run `run_simulation_jounce()` before `run_simulation_roll()`.")
        jounce = self.input_data['jounce']
        RC_y = self.outputs['roll_center_y']
        RC_z = self.outputs['roll_center_z']
        zero_jounce = np.argmin(abs(jounce))
        y_RC0 = RC_y[zero_jounce]
        z_RC0 = RC_z[zero_jounce]
        
        time = np.linspace(0, 2, 4000)
        roll_deg_profile = self.input_sweep(time, -roll_travel, roll_travel)
        phi = np.radians(roll_deg_profile)
        half_track = 0.6096
        y_FL = +half_track
        y_ref = y_RC0
        fl_jounce_from_roll = -np.tan(phi) * (y_FL - y_ref) 
        
        roll_input = np.zeros(time.shape[0],
        dtype=[('time', np.float64), ('jounce', np.float64), ('rack_input', np.float64)]
        )
        roll_input['time'] = time
        roll_input['jounce'] = fl_jounce_from_roll
        roll_input['rack_input'] = 0.0
        
        roll_res = self.fmu_sim.simulate(
        fmi_type="ModelExchange",
        start_time=0,
        stop_time=2,
        step_size=0.00004,
        input_data=roll_input,
        output_vars=[
            'camber', 'toe', 'caster', 'kpi',
            'trail', 'scrub',
            'FVIC_y_migration', 'FVIC_z_migration',
            'roll_center_y', 'roll_center_z'
        ],
        debug_logging=False,
        start_values={'jounce': 0.0, 'rack_input': 0.0},
        relative_tolerance=1e-5
        )
        
        self.rt = roll_res['time'] #roll time
        self.roll_deg = np.interp(self.rt, time, roll_deg_profile) #alrernative for jounce_interp but for roll

        self.roll_outputs = {
            'time': self.rt,
            'roll_deg': self.roll_deg,
            'camber_deg': np.degrees(roll_res['camber']),
            'toe_deg': -np.degrees(roll_res['toe']),
            'caster_deg': np.degrees(roll_res['caster']),
            'kpi_deg': np.degrees(roll_res['kpi']),
            'trail_mm': roll_res['trail'] * 1000.0,
            'scrub_mm': roll_res['scrub'] * 1000.0,
            'FVIC_y_mm': roll_res['FVIC_y_migration'] * 1000.0,
            'FVIC_z_mm': roll_res['FVIC_z_migration'] * 1000.0,
            'RC_y_mm': roll_res['roll_center_y'] * 1000.0,
            'RC_z_mm': roll_res['roll_center_z'] * 1000.0,
        }

        self.roll_deg_fmu = np.interp(self.roll_outputs['time'], time, roll_deg_profile)


    def get_output_jounce(self, name: str, degrees: bool = False):
        if self.outputs is None:
            raise RuntimeError("No sim data. Run the sim again.")
        if name not in self.outputs.dtype.names:
            raise KeyError(f"'{name}' Not one of the outputs.")
        data = self.outputs[name]
        return np.degrees(data) if degrees else data

    def get_output_roll(self, var_name: str):  
        if self.roll_outputs is None:
            raise RuntimeError("No roll data available. Run `run_simulation_roll()` first.")
        if var_name not in self.roll_outputs:
            raise KeyError(f"'{var_name}' not found in roll outputs.")
        data = self.roll_outputs[var_name]
        return data

    # def get_jounce_interp_pair(self, var, degrees=False):
    #     t = self.time
    #     jounce_at_outputs = np.interp(
    #         t,
    #         self.input_data['time'],
    #         self.input_data['jounce']
    #     )
    #     mask = (t >= 0.5) & (t <= 1.5)

    #     xp = jounce_at_outputs[mask]
    #     fp = self.outputs[var][mask]

    #     if degrees:
    #         fp = np.degrees(fp)

    #     xp_unique, idx = np.unique(xp, return_index=True)
    #     fp_unique = fp[idx]

    #     return xp_unique, fp_unique

    # def get_roll_interp_pair(self, var):

    #     name_map = {
    #         'camber':        'camber_deg',
    #         'toe':           'toe_deg',
    #         'caster':        'caster_deg',
    #         'kpi':           'kpi_deg',
    #         'trail':         'trail_mm',
    #         'scrub':         'scrub_mm',
    #         'FVIC_y_migration': 'FVIC_y_mm',
    #         'FVIC_z_migration': 'FVIC_z_mm',
    #         'roll_center_y': 'RC_y_mm',
    #         'roll_center_z': 'RC_z_mm',

    #         'camber_deg':    'camber_deg',
    #         'toe_deg':       'toe_deg',
    #         'caster_deg':    'caster_deg',
    #         'kpi_deg':       'kpi_deg',
    #         'trail_mm':      'trail_mm',
    #         'scrub_mm':      'scrub_mm',
    #         'FVIC_y_mm':     'FVIC_y_mm',
    #         'FVIC_z_mm':     'FVIC_z_mm',
    #         'RC_y_mm':       'RC_y_mm',
    #         'RC_z_mm':       'RC_z_mm',

    #         'roll_deg':      'roll_deg'
    #     }

    #     if var not in name_map:
    #         raise KeyError(f"Unknown roll variable '{var}'. Available: {list(name_map.keys())}")

    #     key = name_map[var]

    #     xp = self.roll_outputs['roll_deg']

    #     fp = self.roll_outputs[key]

    #     xp_unique, idx = np.unique(xp, return_index=True)
    #     fp_unique = fp[idx]

    #     order = np.argsort(xp_unique)
    #     xp_sorted = xp_unique[order]
    #     fp_sorted = fp_unique[order]

    #     return xp_sorted, fp_sorted
    
    def camber_jounce(self): return self.get_output_jounce('camber')
    def toe_jounce(self): return self.get_output_jounce('toe')
    def caster_jounce(self): return self.get_output_jounce('caster')
    def kpi_jounce(self): return self.get_output_jounce('kpi')
    def trail_jounce(self): return self.get_output_jounce('trail')
    def scrub_jounce(self): return self.get_output_jounce('scrub')
    def fvic_y_jounce(self): return self.get_output_jounce('FVIC_y_migration')
    def fvic_z_jounce(self): return self.get_output_jounce('FVIC_z_migration')
    def roll_center_y_jounce(self): return self.get_output_jounce('roll_center_y')
    def roll_center_z_jounce(self): return self.get_output_jounce('roll_center_z')
    
    def camber_roll(self): return self.get_output_roll('camber_deg')
    def toe_roll(self): return self.get_output_roll('toe_deg')
    def caster_roll(self): return self.get_output_roll('caster_deg')
    def kpi_roll(self): return self.get_output_roll('kpi_deg')
    def trail_roll(self): return self.get_output_roll('trail_mm')
    def scrub_roll(self): return self.get_output_roll('scrub_mm')
    def fvic_y_roll(self): return self.get_output_roll('FVIC_y_mm')
    def fvic_z_roll(self): return self.get_output_roll('FVIC_z_mm')
    def roll_center_y_roll(self): return self.get_output_roll('RC_y_mm')
    def roll_center_z_roll(self): return self.get_output_roll('RC_z_mm')
    def roll_angle(self): return self.get_output_roll('roll_deg')
