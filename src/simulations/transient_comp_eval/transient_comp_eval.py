from src.simulations.comp_eval.comp_eval import CompEval
from src._3_custom_libraries.simulation import Simulation
from datetime import datetime
from PIL import Image

import matplotlib.gridspec as gridspec
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import tzlocal
import os


class TransientCompEval(Simulation):
    def __init__(self, model_path: str):
        super().__init__(model_path)

        # mass properties
        self.mass = self.sus_data.total_mass

        # inertia tensor components
        I_tensor = np.array(self.sus_data.inertia_tensor)
        self.I_z = I_tensor[2][2]  # yaw inertia

        # tire contact patches
        front_cp = self.sus_data.FL_tire.contact_patch
        rear_cp = self.sus_data.RL_tire.contact_patch
        self.wheelbase = abs(rear_cp[0] - front_cp[0])

        cg_pos = self.sus_data.CG_node.position

        # distance from center of gravity to axles
        self.l_f = abs(cg_pos[0] - front_cp[0])  # CG to front axle
        self.l_r = abs(cg_pos[0] - rear_cp[0])   # CG to rear axle

        # track widths
        left_cp = self.sus_data.FL_tire.contact_patch[1]
        right_cp = self.sus_data.FR_tire.contact_patch[1]
        self.track_f = abs(left_cp - right_cp)

        left_cp_r = self.sus_data.RL_tire.contact_patch[1]
        right_cp_r = self.sus_data.RR_tire.contact_patch[1]
        self.track_r = abs(left_cp_r - right_cp_r)
        self.track = (self.track_f + self.track_r) / 2

        # center of gravity height
        self.h_cg = cg_pos[2]
        self.R_tire = self.sus_data.FL_tire.outer_diameter / 2

        # ========== TODO: TIRE PROPERTIES ==========
        # TODO: Extract cornering stiffness from MF52 tire model
        # TODO: Calculate peak friction coefficient from tire data
        # For now, using placeholder values

        # ========== TODO: WEIGHT DISTRIBUTION ==========
        # TODO: Calculate front/rear weight distribution
        # TODO: Calculate normal forces per tire
        # TODO: Calculate maximum yaw moment
        # TODO: Calculate maximum yaw acceleration (alpha_z_max)
        # TODO: Calculate understeer gradient

        # ========== GGV PARAMETERS (matching comp_eval) ==========
        self.V_MAX = 31.1
        self.X_OFFSET = 0.5
        self.SCALE_FACTOR = 0.01

        # EVENT-SPECIFIC PARAMETERS
        self.SKIDPAD_RADIUS = 10.6  # meters
        self.DT = 0.01  # timestep (seconds)

        # FIGURE-8 PARAMETERS
        self.CIRCLE_SEPARATION = 3.0  # straight section between circles
        self.FIGURE8_LAPS = 1  # one complete figure-8

        # Make a fake static CompEval instance to reuse its GGV methods
        # Create a CompEval instance to access its methods without re-running simulations
        self._qss = CompEval.__new__(CompEval)  # Create instance without __init__
        self._qss.V_MAX = self.V_MAX
        self._qss.X_OFFSET = self.X_OFFSET
        self._qss.SCALE_FACTOR = self.SCALE_FACTOR


        print(f"\n{'='*50}")
        print(f"QTS Setup - Basic Parameters")
        print(f"{'='*50}")
        print(f"Vehicle: {self.sus_data.vehicle_name}")
        print(f"Mass:     {self.mass:.1f} kg")
        print(f"Yaw Inertia (I_z): {self.I_z:.1f} kg·m²")
        print(f"Wheelbase: {self.wheelbase:.3f} m")
        print(f"l_f: {self.l_f:.3f} m, l_r: {self.l_r:.3f} m")
        print(f"{'='*50}\n")

        print("Running QTS Simulations...")

        print("  Figure-8 skidpad (QTS)...")
        self.figure8_results = self.run_figure8_qts()
        print(f"    Time: {self.figure8_results['time']:.3f}s")

        print("\nReport generation not yet implemented")


    # ========== REUSE GGV METHODS FROM QSS ==========

    def ggv_max_accel(self, v, direction=0):
        """Reuse from CompEval"""
        return self._qss.ggv_max_accel(v, direction)

    def ggv_envelope(self, v):
        """Reuse from CompEval"""
        return self._qss.ggv_envelope(v)

    def ggv_max_long_accel(self, v):
        """Reuse from CompEval"""
        return self._qss.ggv_max_long_accel(v)

    def ggv_max_lat_accel(self, v):
        """Reuse from CompEval"""
        return self._qss.ggv_max_lat_accel(v)


    # ========== QTS SIMULATION METHODS ==========

    def run_figure8_qts(self):
        """
        QTS simulation of figure-8 skidpad with yaw dynamics.
        TODO: Implement full QTS iteration
        """
        print("    [QTS implementation pending...]")

        return {
            'time': 0.0,
            'path_s': np.array([0]),
            'path_k': np.array([0]),
            'v_profile': np.array([0]),
            'omega_history': np.array([0]),
            'alpha_f_history': np.array([0]),
            'alpha_r_history': np.array([0]),
            'x_history': np.array([0]),
            'y_history': np.array([0])
        }

    # TODO: Implement define_figure8_path()
    # TODO: Implement calculate_slip_angles()
    # TODO: Implement calculate_yaw_moment()


    # ========== PLOTTING METHODS ==========

    def plot_title_page(self):
        """Title page"""
        fig = plt.figure(figsize=(14, 8.5), dpi=300)

        local_tz = tzlocal.get_localzone()
        now = datetime.now(local_tz)

        logo_path = "./src/_4_ico/lhrEnobackground.png"
        if os.path.exists(logo_path):
            img = Image.open(logo_path)
            img_resized = img.resize((int(img.width * 0.35), int(img.height * 0.35)), Image.Resampling.LANCZOS)
            img_np = mpimg.pil_to_array(img_resized)
            fig_height_px = fig.get_figheight() * fig.dpi
            fig.figimage(img_np, xo=10, yo=int(fig_height_px * 0.815), zorder=1)

        fig.text(0.5, 0.55, "Transient Component Evaluation (QTS) Report", fontsize=30, ha='center')
        fig.text(0.5, 0.50, "Simulation Author: Arjun Rao", fontsize=12, ha='center')
        fig.text(0.5, 0.45, f"Generated By: {self.get_git_name()} ({self.get_git_username()})", fontsize=12, ha='center')
        fig.text(0.5, 0.40, f"Date: {now.strftime('%Y-%m-%d, %I:%M %p %Z')}", fontsize=12, ha='center')

        fig.gca().axis('off')
        return fig

    def plot_ggv_surface(self):
        """3D GGV surface"""
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        ax = fig.add_subplot(111, projection='3d')

        for v in np.linspace(5, self.V_MAX, 8):
            ax_env, ay_env = self.ggv_envelope(v)
            v_arr = np.full_like(ax_env, v)
            ax.plot(ax_env, ay_env, v_arr, linewidth=1.5, alpha=0.7)

        ax.set_xlabel('ax (m/s^2)')
        ax.set_ylabel('ay (m/s^2)')
        ax.set_zlabel('v (m/s)')
        ax.set_title('GGV Surface', fontsize=16, pad=20)

        fig.text(0.5, 0.02, f"GGV: (ax_g+{self.X_OFFSET})^2 + (ay_g)^2 = 9*({self.SCALE_FACTOR}*v^2 - v^3/4000)",
                 fontsize=10, ha='center')

        return fig

    def plot_gg_envelopes(self):
        """G-G diagrams at different speeds"""
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(2, 3)

        for i, v in enumerate([5, 10, 15, 20, 25, 30]):
            ax = fig.add_subplot(gs[i // 3, i % 3])
            if v <= self.V_MAX:
                ax_env, ay_env = self.ggv_envelope(v)
                ax.plot(ay_env, ax_env, 'b-', linewidth=2)
                ax.fill(ay_env, ax_env, alpha=0.2)
            ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')
            ax.axvline(0, color='gray', linewidth=0.5, linestyle='--')
            ax.set_xlabel('ay (m/s^2)', fontsize=8)
            ax.set_ylabel('ax (m/s^2)', fontsize=8)
            ax.set_title(f'v = {v} m/s', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')

        fig.suptitle('G-G Envelopes', fontsize=16, y=0.98)
        fig.tight_layout()
        return fig

    # TODO: Add new QTS-specific plots:
    # - plot_figure8_trajectory()
    # - plot_figure8_dynamics()
    # - plot_vehicle_params()