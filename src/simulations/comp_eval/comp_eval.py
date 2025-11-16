
from src._3_custom_libraries.simulation import Simulation
from datetime import datetime
from PIL import Image

import matplotlib.gridspec as gridspec
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import tzlocal
import os


class CompEval(Simulation):
    def __init__(self, model_path: str):
        super().__init__(model_path=model_path)

        self.V_MAX = 31.1 # meters
        self.X_OFFSET = 0.5
        self.SCALE_FACTOR = 0.01

        self.ACCEL_DISTANCE = 75.0 # meters
        self.SKIDPAD_RADIUS = 10.6 # meters
        self.SKIDPAD_LAPS = 2

        self.DT = 0.01  # timestep (seconds)

        print("Running GGV Simulation...")

        print("  Acceleration event...")
        self.accel_results = self.run_acceleration()
        print(f"    Time: {self.accel_results['time']:.3f}s")

        print("  Skidpad event...")
        self.skidpad_results = self.run_skidpad()
        print(f"    Time: {self.skidpad_results['time']:.3f}s, {self.skidpad_results['lat_g']:.2f}G")

        print("\nGenerating PDF report...")
        plots = []


        plots.append(self.plot_title_page())

        plots.append(self.plot_ggv_surface())
        plots.append(self.plot_gg_envelopes())

        plots.append(self.plot_summary())
        plots.append(self.plot_acceleration())
        plots.append(self.plot_skidpad())

        self._generate_pdf(figs=plots, save_path="./src/simulations/comp_eval/comp_eval_outputs/comp_eval_report.pdf")
        print("\n✓ Done! Report saved: ./src/simulations/comp_eval/comp_eval_outputs/comp_eval_report.pdf")

        with open("./src/simulations/comp_eval/comp_eval_outputs/debug.txt", "w") as f:
            f.write("=== GGV DIAGNOSTICS ===\n")
            f.write(f"V_MAX = {self.V_MAX} m/s\n")
            f.write(f"X_OFFSET = {self.X_OFFSET}\n")
            f.write(f"SCALE_FACTOR = {self.SCALE_FACTOR}\n\n")

            f.write("GGV values:\n")
            f.write(f"  v=0.1 m/s: ax_max={self.ggv_max_long_accel(0.1):.4f} m/s^2, ay_max={self.ggv_max_lat_accel(0.1):.4f} m/s^2\n")
            f.write(f"  v=10 m/s: ax_max={self.ggv_max_long_accel(10):.4f} m/s^2, ay_max={self.ggv_max_lat_accel(10):.4f} m/s^2\n")
            f.write(f"  v=20 m/s: ax_max={self.ggv_max_long_accel(20):.4f} m/s^2, ay_max={self.ggv_max_lat_accel(20):.4f} m/s^2\n\n")

            f.write("ACCELERATION EVENT:\n")
            f.write(f"  Time: {self.accel_results['time']:.3f} s\n")
            f.write(f"  First 10 velocities: {self.accel_results['v_history'][:10]}\n")
            f.write(f"  Last 5 velocities: {self.accel_results['v_history'][-5:]}\n")
            f.write(f"  First 10 accelerations: {self.accel_results['a_history'][:10]}\n")
            f.write(f"  Max velocity: {max(self.accel_results['v_history']):.4f} m/s\n\n")

            f.write("SKIDPAD EVENT:\n")
            f.write(f"  Time: {self.skidpad_results['time']:.3f} s\n")
            f.write(f"  Optimal velocity: {self.skidpad_results['v_opt']:.4f} m/s\n")
            f.write(f"  Lateral accel: {self.skidpad_results['ay']:.4f} m/s^2\n")
            f.write(f"  Lateral G: {self.skidpad_results['lat_g']:.4f} G\n")

        print("  Debug info written to: ./src/simulations/comp_eval/comp_eval_outputs/debug.txt")


    def ggv_max_accel(self, v, direction=0):
        """Get max acceleration at velocity v in direction (radians)"""
        if v < 0.1:
            return 0.0, 0.0
        radius_squared = 9*(self.SCALE_FACTOR * v**2 - v**3 / 4000.0)  # v^3/4000
        if radius_squared < 0:
            return 0.0, 0.0 # no grip, prolyl spin
        radius = np.sqrt(radius_squared)
        ax = (radius * np.cos(direction) - self.X_OFFSET) * 9.81  # Convert G to m/s^2
        ay = radius * np.sin(direction) * 9.81  # Convert G to m/s^2
        return ax, ay

    def ggv_envelope(self, v):
        """Get full envelope at velocity v"""
        if v < 0.1:
            return np.array([0.0]), np.array([0.0])
        radius_squared = 9*(self.SCALE_FACTOR * v**2 - v**3 / 4000.0)
        if radius_squared < 0:
            return np.array([0.0]), np.array([0.0])
        radius = np.sqrt(radius_squared)
        theta = np.linspace(0, 2*np.pi, 100)
        ax = (radius * np.cos(theta) - self.X_OFFSET) * 9.81  # Convert G to m/s^2
        ay = radius * np.sin(theta) * 9.81  # Convert G to m/s^2
        return ax, ay

    def ggv_max_long_accel(self, v):
        """Max forward acceleration - don't apply X_OFFSET here, it's for the full envelope"""
        if v < 0.1:
            v = 0.1  # no zero for math
        radius_squared = 9*(self.SCALE_FACTOR * v**2 - v**3 / 4000.0)
        if radius_squared < 0:
            return 0.0
        radius = np.sqrt(radius_squared)
        ax = (radius + self.X_OFFSET) * 9.81
        return ax

    def ggv_max_lat_accel(self, v):
        """Max lateral acceleration"""
        if v < 0.1:
            v = 0.1
        radius_squared = 9*(self.SCALE_FACTOR * v**2 - v**3 / 4000.0)
        if radius_squared < 0:
            return 0.0
        radius = np.sqrt(radius_squared)
        ay = radius * 9.81
        return ay


    def run_acceleration(self):
        """Simulate 75m acceleration"""
        x, vx = 0.0, 0.01
        t_history, x_history, v_history, a_history = [0], [0], [0.01], [0]
        t = 0.0

        while x < self.ACCEL_DISTANCE and t < 15.0:
            ax = max(0, self.ggv_max_long_accel(abs(vx)))
            x += vx * self.DT + 0.5 * ax * self.DT**2
            vx += ax * self.DT
            t += self.DT

            t_history.append(t)
            x_history.append(x)
            v_history.append(vx)
            a_history.append(ax)

        return {
            'time': t,
            't_history': np.array(t_history),
            'x_history': np.array(x_history),
            'v_history': np.array(v_history),
            'a_history': np.array(a_history)
        }

    def run_skidpad(self):
        """Simulate skidpad - find optimal constant velocity"""
        v_min, v_max = 0.1, self.V_MAX

        while v_max - v_min > 0.01:
            v_test = (v_min + v_max) / 2
            ay_required = v_test**2 / self.SKIDPAD_RADIUS
            ay_available = abs(self.ggv_max_lat_accel(v_test))

            if ay_required <= ay_available:
                v_min = v_test
            else:
                v_max = v_test

        v_opt = v_min
        distance = 2 * np.pi * self.SKIDPAD_RADIUS * self.SKIDPAD_LAPS
        time = distance / v_opt
        ay = v_opt**2 / self.SKIDPAD_RADIUS

        # Generate trajectory
        n = int(time / self.DT)
        theta = np.linspace(0, 2*np.pi*self.SKIDPAD_LAPS, n)
        x = self.SKIDPAD_RADIUS * np.cos(theta)
        y = self.SKIDPAD_RADIUS * np.sin(theta)

        return {
            'time': time,
            'v_opt': v_opt,
            'ay': ay,
            'lat_g': ay / 9.81,
            'x': x,
            'y': y
        }


    def plot_title_page(self):
        """Title page"""
        fig = plt.figure(figsize=(14, 8.5), dpi=300)

        local_tz = tzlocal.get_localzone()
        now = datetime.now(local_tz)

        # Logo (optional)
        logo_path = "./src/_4_ico/lhrEnobackground.png"
        if os.path.exists(logo_path):
            img = Image.open(logo_path)
            img_resized = img.resize((int(img.width * 0.35), int(img.height * 0.35)), Image.Resampling.LANCZOS)
            img_np = mpimg.pil_to_array(img_resized)
            fig_height_px = fig.get_figheight() * fig.dpi
            fig.figimage(img_np, xo=10, yo=int(fig_height_px * 0.815), zorder=1)

        fig.text(0.5, 0.55, "Component Evaluation (GGV) Report", fontsize=30, ha='center')
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

        # Add equation at bottom (in Gs)
        fig.text(0.5, 0.02, f"GGV: (ax_g+{self.X_OFFSET})^2 + (ay_g)^2 = 9*({self.SCALE_FACTOR}*v^2 - v^3/4000)  |  accel = G * 9.81",
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

    def plot_summary(self):
        """Summary table"""
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        ax = fig.add_subplot(111)
        ax.axis('off')

        data = [
            ['Event', 'Metric', 'Value'],
            ['Acceleration (75m)', 'Time', f"{self.accel_results['time']:.3f} s"],
            ['', 'Max Velocity', f"{max(self.accel_results['v_history']):.2f} m/s"],
            ['Skidpad', 'Time', f"{self.skidpad_results['time']:.3f} s"],
            ['', 'Optimal Velocity', f"{self.skidpad_results['v_opt']:.2f} m/s"],
            ['', 'Lateral G', f"{self.skidpad_results['lat_g']:.2f} G"]
        ]

        table = ax.table(cellText=data, loc='center', cellLoc='center', colWidths=[0.3, 0.3, 0.4])
        table.scale(1, 4)
        table.auto_set_font_size(False)
        table.set_fontsize(12)

        for j in range(3):
            table[0, j].set_facecolor('#4472C4')
            table[0, j].set_text_props(weight='bold', color='white')

        fig.text(0.5, 0.85, 'Performance Summary', fontsize=20, ha='center', weight='bold')
        return fig

    def plot_acceleration(self):
        """Acceleration event plots"""
        res = self.accel_results
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(2, 2)

        # Velocity vs Time
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(res['t_history'], res['v_history'], 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.set_title('Velocity vs Time')
        ax1.grid(True, alpha=0.3)

        # Distance vs Time
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(res['t_history'], res['x_history'], 'r-', linewidth=2)
        ax2.axhline(75, color='gray', linestyle='--', label='Finish')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Distance (m)')
        ax2.set_title('Distance vs Time')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Acceleration vs Time
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(res['t_history'], res['a_history'], 'g-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Accel (m/s^2)')
        ax3.set_title('Acceleration vs Time')
        ax3.grid(True, alpha=0.3)

        # Velocity vs Distance
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(res['x_history'], res['v_history'], 'm-', linewidth=2)
        ax4.set_xlabel('Distance (m)')
        ax4.set_ylabel('Velocity (m/s)')
        ax4.set_title('Velocity vs Distance')
        ax4.grid(True, alpha=0.3)

        fig.suptitle(f"Acceleration Event - {res['time']:.3f}s", fontsize=16, y=0.98)
        fig.tight_layout()
        return fig

    def plot_skidpad(self):
        """Skidpad event plots"""
        res = self.skidpad_results
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(1, 2, width_ratios=[0.6, 0.4])

        # Trajectory
        ax1 = fig.add_subplot(gs[0])
        ax1.plot(res['x'], res['y'], 'b-', linewidth=2)
        ax1.plot(res['x'][0], res['y'][0], 'go', markersize=10, label='Start')
        ax1.plot(res['x'][-1], res['y'][-1], 'ro', markersize=10, label='Finish')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Skidpad Trajectory')
        ax1.set_aspect('equal')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Metrics table
        ax2 = fig.add_subplot(gs[1])
        ax2.axis('off')

        data = [
            ['Metric', 'Value'],
            ['Time', f"{res['time']:.3f} s"],
            ['Velocity', f"{res['v_opt']:.2f} m/s"],
            ['Lateral Accel', f"{res['ay']:.2f} m/s^2"],
            ['Lateral G', f"{res['lat_g']:.2f} G"],
            ['Radius', f"{self.SKIDPAD_RADIUS:.2f} m"]
        ]

        table = ax2.table(cellText=data, loc='center', cellLoc='left', colWidths=[0.5, 0.5])
        table.scale(1, 3)
        table.auto_set_font_size(False)
        table.set_fontsize(11)

        for j in range(2):
            table[0, j].set_facecolor('#4472C4')
            table[0, j].set_text_props(weight='bold', color='white')

        fig.suptitle(f"Skidpad Event - {res['time']:.3f}s", fontsize=16, y=0.96)
        fig.tight_layout()
        return fig
