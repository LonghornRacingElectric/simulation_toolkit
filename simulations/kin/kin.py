from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension
from _4_custom_libraries.misc_math import rotation_matrix
from _4_custom_libraries.cache import SISO_cache

from typing import Callable, MutableSequence, Sequence, Set, Tuple
from scipy.interpolate import RegularGridInterpolator
from matplotlib.backends.backend_pdf import PdfPages
from scipy.interpolate import CubicSpline
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from dataclasses import dataclass
from datetime import datetime
from PIL import Image

import matplotlib.gridspec as gridspec
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import tzlocal
import pickle
import yaml

from copy import deepcopy


class Kinematics:
    def __init__(self, model_path: str):
        self.sus_data: SuspensionData = SuspensionData(path=model_path)
        self.sus: Suspension = Suspension(sus_data=self.sus_data)
        self.sus_copy = deepcopy(self.sus)

        roll_n_steps = 1

        with open("./simulations/kin/kin_inputs/kin.yml") as f:
            try:
                self.plot_config: dict[str, dict[str, dict]] = yaml.safe_load(f)
                self.FMU_config: dict[str, dict] = self.plot_config.pop("FMU Settings")
                self.fit_config: dict[str, dict] = self.plot_config.pop("Fit Settings")
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)

        # Generate FMU
        if self.FMU_config["Generate"]:
            FMU_refinement = self.FMU_config["Refinement"]
            
            # Sweeps
            hwa_sweep = np.linspace(-self.FMU_config["Hwa Sweep"], self.FMU_config["Hwa Sweep"], FMU_refinement)
            heave_sweep = np.linspace(-self.FMU_config["Heave Sweep"], self.FMU_config["Heave Sweep"], FMU_refinement) * 0.0254
            pitch_sweep = np.linspace(-self.FMU_config["Pitch Sweep"], self.FMU_config["Pitch Sweep"], FMU_refinement)
            roll_sweep = np.linspace(-self.FMU_config["Roll Sweep"], self.FMU_config["Roll Sweep"], FMU_refinement)

            state_tracking: dict[str, MutableSequence] = {x: [] for x in self.sus_copy.state.keys()}
            
            eval_num = 0
            total_evals = FMU_refinement**4

            for hwa in hwa_sweep:
                for heave in heave_sweep:
                    for pitch in pitch_sweep:
                        for roll in roll_sweep:
                            eval_num += 1
                            print(f"FMU Generation Progress: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")

                            self.sus_copy = deepcopy(self.sus)
                            self.sus_copy.steer(hwa=hwa)
                            self.sus_copy.heave(heave=heave)
                            self.sus_copy.pitch(pitch=pitch)
                            self.sus_copy.roll(roll=roll)

                            for key in self.sus_copy.state.keys():
                                state_tracking[key].append(float(self.sus_copy.state[key]))
                                
                            
                            self.sus_copy = deepcopy(self.sus)

            FMU_fits: dict[str, RegularGridInterpolator] = {}
            
            for key in state_tracking.keys():
                output_state = np.array(state_tracking[key]).reshape((FMU_refinement, FMU_refinement, FMU_refinement, FMU_refinement))
                
                if self.FMU_config["Extrapolate"]:
                    interp_func = RegularGridInterpolator((hwa_sweep, heave_sweep, pitch_sweep, roll_sweep), output_state,
                                                          bounds_error=False, fill_value=None)
                else:
                    interp_func = RegularGridInterpolator((hwa_sweep, heave_sweep, pitch_sweep, roll_sweep), output_state)
                
                FMU_fits[key] = interp_func
            
            FMU_fits["keys"] = list(FMU_fits.keys())
            
            with open("./simulations/kin/kin_outputs/kin_FMU.pkl", 'wb') as f:
                pickle.dump(FMU_fits, f)
            
            print()

        # Make cover page :D
        
        local_tz = tzlocal.get_localzone() # Get local time zone
        now = datetime.now(local_tz) # Get current local time with time zone
        
        if self.fit_config["Generate Linear"] or self.fit_config["Generate Cubic"]:
            title_fig = plt.figure(figsize=(14, 8.5), dpi=300)
            
            # Logo bullshit (this was all trial and error)
            logo_path = "_5_ico/lhrEnobackground.png"
            img = Image.open(logo_path) # Read image
            img_resized = img.resize((int(img.width * 0.35), int(img.height * 0.35)), Image.Resampling.LANCZOS) # Resize image
            img_np = mpimg.pil_to_array(img_resized) # Convert to array
            fig_height_px = title_fig.get_figheight() * title_fig.dpi # Get figure sizing
            title_fig.figimage(img_np, xo=10, yo=int(fig_height_px * 0.815), zorder=1) # Insert image

            # Insert text
            title_fig.text(0.5, 0.55, "Kinematics Report", fontsize=30, ha='center')
            title_fig.text(0.5, 0.50, "Simulation Author: Robert Horvath", fontsize=12, ha='center')
            title_fig.text(0.5, 0.45, f"Generated By: {self.get_git_name()} ({self.get_git_username()})", fontsize=12, ha='center')
            title_fig.text(0.5, 0.40, f"Date: {now.strftime("%Y-%m-%d, %I:%M %p %Z")}", fontsize=12, ha='center')

            notes = []

            if self.fit_config["Generate Linear"]:
                notes.append("Linear fits are tangent lines about x = 0 (NOT fits over the entire range)")
            if self.fit_config["Generate Cubic"]:
                notes.append("Cubic fits are performed over the entire visible domain (fits over the entire range)")
            
            for index, note in enumerate(notes[::-1]):
                title_fig.text(0.02, 0.025 * index + 0.025, f"Note {(len(notes) - index)}: {note}", fontsize=8, ha='left')
            
            title_fig.gca().axis('off')

        else:
            title_fig = plt.figure(figsize=(11, 8.5), dpi=300)
            
            # Logo bullshit (this was all trial and error)
            logo_path = "_5_ico/lhrEnobackground.png"
            img = Image.open(logo_path) # Read image
            img_resized = img.resize((int(img.width * 0.35), int(img.height * 0.35)), Image.Resampling.LANCZOS) # Resize image
            img_np = mpimg.pil_to_array(img_resized) # Convert to array
            fig_height_px = title_fig.get_figheight() * title_fig.dpi # Get figure sizing
            title_fig.figimage(img_np, xo=10, yo=int(fig_height_px * 0.815), zorder=1) # Insert image

            # Insert text
            title_fig.text(0.5, 0.55, "Kinematics Report", fontsize=30, ha='center')
            title_fig.text(0.5, 0.5, "Simulation Author: Robert Horvath", fontsize=12, ha='center')
            title_fig.text(0.5, 0.45, f"Generated By: {self.get_git_name()} ({self.get_git_username()})", fontsize=12, ha='center')
            title_fig.text(0.5, 0.4, f"Date: {now.strftime("%Y-%m-%d, %I:%M %p %Z")}", fontsize=12, ha='center')
            title_fig.gca().axis('off')
        
        # Generate Plots (from FMU and kin model)
        self.outputs: Set[str] = set([key[key.index("_") + 1:] for key in self.sus.state.keys()])

        plots = []
        plots.append(title_fig)

        total_evals = sum([x["x-axis"]["Number Steps"] for x in self.plot_config.values()])
        eval_num = 0

        if self.FMU_config["Evaluate"]:
            with open("./simulations/kin/kin_outputs/kin_FMU.pkl", 'rb') as f:
                FMU_fits = pickle.load(f)

        for _, value in self.plot_config.items():
            num_plots = [x[1] for x in value["Corners"].items()].count(True)
            if num_plots == 2:
                axle_vals = [[], []]
                if self.fit_config["Generate Linear"] or self.fit_config["Generate Cubic"]:
                    fig = plt.figure(figsize=(14, 8.5), dpi=300)
                    gs = gridspec.GridSpec(1, 3, width_ratios=[1, 1, 0.75])
                
                    ax1 = fig.add_subplot(gs[0])
                    ax2 = fig.add_subplot(gs[1])

                else:
                    fig, axes = plt.subplots(nrows=1, ncols=2)
                    fig.set_size_inches(w=11, h=8.5)
                    fig.suptitle(value["Title"], fontsize=18)

                    ax1 = axes[0]
                    ax2 = axes[1]

                if value["x-axis"]["Label"].lower() == "jounce":
                    jounce_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])

                    if value["x-axis"]["Unit"].lower() == "mm":
                        sweep = jounce_sweep / 1000
                    else:
                        sweep = jounce_sweep
                    
                    for jounce_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self = self.heave(jounce_val)
                        
                        for _, val in value["y-axis"]["Outputs"].items():
                            if (val not in self.outputs) and not (val == None):
                                raise Exception(f"Invalid output specified in {value["Title"]} ({val}). Please use one of the following: {", ".join(self.outputs)}")
                            
                        axle_vals[0].append(self.sus.state["Fr_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        axle_vals[1].append(self.sus.state["Rr_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                    
                    ax1.set_title(f"Fr {value["Title"]}", fontsize=14)
                    ax2.set_title(f"Rr {value["Title"]}", fontsize=14)
                    
                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    ax1.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    ax2.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    ax1.set_ylabel(f"Fr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    ax2.set_ylabel(f"Rr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)

                    ax1.plot(jounce_sweep, axle_vals[0])
                    ax2.plot(jounce_sweep, axle_vals[1])

                    if self.FMU_config["Evaluate"]:
                        Fr_FMU_vals = np.array([FMU_fits["Fr_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, jounce, 0, 0]))[0] for jounce in sweep])
                        Rr_FMU_vals = np.array([FMU_fits["Rr_" + value["y-axis"]["Outputs"]["RL"]](np.array([0, jounce, 0, 0]))[0] for jounce in sweep])
                        
                        ax1.plot(jounce_sweep, Fr_FMU_vals * value["y-axis"]["Multipliers"]["FL"], linestyle='--')
                        ax2.plot(jounce_sweep, Rr_FMU_vals * value["y-axis"]["Multipliers"]["RL"], linestyle='--')

                    slope_0, func_0 = self.nom_tangent(a=axle_vals[0], b=jounce_sweep)
                    slope_1, func_1 = self.nom_tangent(a=axle_vals[1], b=jounce_sweep)

                    jounce_range = max(jounce_sweep) - min(jounce_sweep)
                    tangent_bounds = [-0.10 * jounce_range, 0.10 * jounce_range]

                    ax1.plot(tangent_bounds, func_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax2.plot(tangent_bounds, func_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        ax1.set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        ax2.set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])

                    ax1.legend(loc='lower center')
                    ax2.legend(loc='lower center')

                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    custom_lines = [Line2D([0], [0], color=colors[0], lw=1.5),
                                    Line2D([0], [0], color=colors[1], lw=1.5, linestyle='--')]

                    fig.legend(custom_lines, ['Full Model', 'FMU'], loc='upper right')

                elif value["x-axis"]["Label"].lower() == "roll":
                    roll_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])

                    if value["x-axis"]["Unit"].lower() == "rad":
                        sweep = roll_sweep * 180 / np.pi
                    else:
                        sweep = roll_sweep

                    for roll_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self = self.roll(roll_val, roll_n_steps)
                        
                        for _, val in value["y-axis"]["Outputs"].items():
                            if (val not in self.outputs) and not (val == None):
                                raise Exception(f"Invalid output specified in {value["Title"]} ({val}). Please use one of the following: {", ".join(self.outputs)}")
                            
                        axle_vals[0].append(self.sus.state["Fr_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        axle_vals[1].append(self.sus.state["Rr_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                    
                    ax1.set_title(f"Fr {value["Title"]}", fontsize=14)
                    ax2.set_title(f"Rr {value["Title"]}", fontsize=14)

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]
                    
                    ax1.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    ax2.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    ax1.set_ylabel(f"Fr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    ax2.set_ylabel(f"Rr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    
                    ax1.plot(roll_sweep, axle_vals[0])
                    ax2.plot(roll_sweep, axle_vals[1])

                    if self.FMU_config["Evaluate"]:
                        Fr_FMU_vals = np.array([FMU_fits["Fr_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, 0, 0, roll]))[0] for roll in sweep])
                        Rr_FMU_vals = np.array([FMU_fits["Rr_" + value["y-axis"]["Outputs"]["RL"]](np.array([0, 0, 0, roll]))[0] for roll in sweep])
                        
                        ax1.plot(roll_sweep, Fr_FMU_vals * value["y-axis"]["Multipliers"]["FL"], linestyle='--')
                        ax2.plot(roll_sweep, Rr_FMU_vals * value["y-axis"]["Multipliers"]["RL"], linestyle='--')
                    
                    slope_0, func_0 = self.nom_tangent(a=axle_vals[0], b=roll_sweep)
                    slope_1, func_1 = self.nom_tangent(a=axle_vals[1], b=roll_sweep)

                    roll_range = max(roll_sweep) - min(roll_sweep)
                    tangent_bounds = [-0.10 * roll_range, 0.10 * roll_range]

                    ax1.plot(tangent_bounds, func_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax2.plot(tangent_bounds, func_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        ax1.set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        ax2.set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])

                    ax1.legend(loc='lower center')
                    ax2.legend(loc='lower center')

                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    custom_lines = [Line2D([0], [0], color=colors[0], lw=1.5),
                                    Line2D([0], [0], color=colors[1], lw=1.5, linestyle='--')]

                    fig.legend(custom_lines, ['Full Model', 'FMU'], loc='upper right')
                    
                if value["Grid"]:
                    ax1.grid()
                    ax2.grid()
                
                if self.fit_config["Generate Linear"] or self.fit_config["Generate Cubic"]:
                    ax_table = fig.add_subplot(gs[:, 2])
                    ax_table.axis("off")

                    table_data = []
                    line1 = ax1.lines[0]
                    line2 = ax2.lines[0]

                    line1_x = line1.get_xdata()
                    line1_y = line1.get_ydata()
                    line2_x = line2.get_xdata()
                    line2_y = line2.get_ydata()

                    if self.fit_config["Generate Linear"]:
                        slope_0, func_0 = self.nom_tangent(a=line1_y, b=line1_x)
                        slope_1, func_1 = self.nom_tangent(a=line2_y, b=line2_x)

                        coeff_0_0 = slope_0
                        coeff_0_1 = float(func_0(0))

                        coeff_1_0 = slope_1
                        coeff_1_1 = float(func_1(0))

                        table_data.append(["Linear Fit", r"$f(x) = a_{1}x + a_{0}$"])
                        table_data.append(["Fr", f"{r'$f(x) = $'}{round(coeff_0_0, 3)}{r'$x + $'}{round(coeff_0_1, 3)}"])
                        table_data.append(["Rr", f"{r'$f(x) = $'}{round(coeff_1_0, 3)}{r'$x + $'}{round(coeff_1_1, 3)}"])
                    
                    if self.fit_config["Generate Cubic"]:
                        if self.fit_config["Generate Linear"]:
                            table_data.append([None, None])
                        a_3, a_2, a_1, a_0 = np.polyfit(line1_x, line1_y, 3)
                        b_3, b_2, b_1, b_0 = np.polyfit(line2_x, line2_y, 3)

                        table_data.append(["Cubic Fit", r"$f(x) = a_{3}x^{3} + a_{2}x^{2} + a_{1}x + a_{0}$"])
                        table_data.append(["Fr", f"{r'$f(x) = $'}{round(a_3, 3)}{r'$x^{3} + $'}{round(a_2, 3)}{r'$x^{2} + $'}{round(a_1, 3)}{r'$x + $'}{round(a_0, 3)}"])
                        table_data.append(["Rr", f"{r'$f(x) = $'}{round(b_3, 3)}{r'$x^{3} + $'}{round(b_2, 3)}{r'$x^{2} + $'}{round(b_1, 3)}{r'$x + $'}{round(b_0, 3)}"])

                    # Create table
                    table = ax_table.table(
                        cellText=table_data,
                        loc="center",
                        cellLoc="center",
                        colWidths=[0.2, 0.8]
                    )

                    table.scale(1, 2.2)
                    table.auto_set_font_size(False)
                    table.set_fontsize(8)

                    # Clean section headers
                    count = 0
                    for i, row in enumerate(table_data):
                        if row[0] == None:
                            table[i, 0].set_visible(False)
                            table[i, 1].set_visible(False)
                            continue

                        if (count % 3) == 0:
                            cell = table[i, 0]
                            cell.set_text_props(weight="bold", ha="center")
                            cell.visible_edges = "open"
                            cell.set_linewidth(0)
                            cell.set_facecolor("#f0f0f0")

                            cell = table[i, 1]
                            cell.set_text_props(weight="bold", ha="center")
                            cell.visible_edges = "open"
                            cell.set_linewidth(0)
                            cell.set_facecolor("#f0f0f0")
                        
                        count += 1
                    
                    fig.tight_layout()
                        
            if num_plots == 4:
                corner_vals = [[], [], [], []]
                if self.fit_config["Generate Linear"] or self.fit_config["Generate Cubic"]:
                    fig = plt.figure(figsize=(14, 8.5), dpi=300)
                    gs = gridspec.GridSpec(2, 3, width_ratios=[1, 1, 0.75])
                
                    ax1 = fig.add_subplot(gs[0, 0])
                    ax2 = fig.add_subplot(gs[0, 1])
                    ax3 = fig.add_subplot(gs[1, 0])
                    ax4 = fig.add_subplot(gs[1, 1])

                else:
                    fig, axes = plt.subplots(nrows=2, ncols=2)
                    fig.set_size_inches(w=11, h=8.5)
                    fig.suptitle(value["Title"], fontsize=18)
                
                    ax1 = axes[0, 0]
                    ax2 = axes[0, 1]
                    ax3 = axes[1, 0]
                    ax4 = axes[1, 1]
                
                if value["x-axis"]["Label"].lower() == "jounce":
                    jounce_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])

                    if value["x-axis"]["Unit"].lower() == "mm":
                        sweep = jounce_sweep / 1000
                    else:
                        sweep = jounce_sweep
                    
                    for jounce_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self = self.heave(jounce_val)
                        
                        for _, val in value["y-axis"]["Outputs"].items():
                            if val not in self.outputs:
                                raise Exception(f"Invalid output specified in {value["Title"]} ({val}). Please use one of the following: {", ".join(self.outputs)}")
                            
                        corner_vals[0].append(self.sus.state["FL_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        corner_vals[1].append(self.sus.state["FR_" + value["y-axis"]["Outputs"]["FR"]] * value["y-axis"]["Multipliers"]["FR"])
                        corner_vals[2].append(self.sus.state["RL_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                        corner_vals[3].append(self.sus.state["RR_" + value["y-axis"]["Outputs"]["RR"]] * value["y-axis"]["Multipliers"]["RR"])
                    
                    ax1.set_title(f"FL {value["Title"]}", fontsize=14)
                    ax2.set_title(f"FR {value["Title"]}", fontsize=14)
                    ax3.set_title(f"RL {value["Title"]}", fontsize=14)
                    ax4.set_title(f"RR {value["Title"]}", fontsize=14)

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    ax1.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    ax2.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    ax3.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    ax4.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    ax1.set_ylabel(f"FL {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    ax2.set_ylabel(f"FR {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    ax3.set_ylabel(f"RL {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    ax4.set_ylabel(f"RR {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)

                    ax1.plot(jounce_sweep, corner_vals[0])
                    ax2.plot(jounce_sweep, corner_vals[1])
                    ax3.plot(jounce_sweep, corner_vals[2])
                    ax4.plot(jounce_sweep, corner_vals[3])

                    if self.FMU_config["Evaluate"]:
                        FL_FMU_vals = []
                        FR_FMU_vals = []
                        RL_FMU_vals = []
                        RR_FMU_vals = []

                        for jounce in sweep:
                            FL_FMU_vals.append(FMU_fits["FL_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, jounce, 0, 0]))[0])
                            FR_FMU_vals.append(FMU_fits["FR_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, jounce, 0, 0]))[0])
                            RL_FMU_vals.append(FMU_fits["RL_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, jounce, 0, 0]))[0])
                            RR_FMU_vals.append(FMU_fits["RR_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, jounce, 0, 0]))[0])

                        ax1.plot(jounce_sweep, np.array(FL_FMU_vals) * value["y-axis"]["Multipliers"]["FL"], linestyle='--')
                        ax2.plot(jounce_sweep, np.array(FR_FMU_vals) * value["y-axis"]["Multipliers"]["FR"], linestyle='--')
                        ax3.plot(jounce_sweep, np.array(RL_FMU_vals) * value["y-axis"]["Multipliers"]["RL"], linestyle='--')
                        ax4.plot(jounce_sweep, np.array(RR_FMU_vals) * value["y-axis"]["Multipliers"]["RR"], linestyle='--')

                    slope_0, func_0 = self.nom_tangent(a=corner_vals[0], b=jounce_sweep)
                    slope_1, func_1 = self.nom_tangent(a=corner_vals[1], b=jounce_sweep)
                    slope_2, func_2 = self.nom_tangent(a=corner_vals[2], b=jounce_sweep)
                    slope_3, func_3 = self.nom_tangent(a=corner_vals[3], b=jounce_sweep)

                    jounce_range = max(jounce_sweep) - min(jounce_sweep)
                    tangent_bounds = [-0.10 * jounce_range, 0.10 * jounce_range]

                    ax1.plot(tangent_bounds, func_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax2.plot(tangent_bounds, func_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax3.plot(tangent_bounds, func_2(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_2, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax4.plot(tangent_bounds, func_3(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_3, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        ax1.set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        ax2.set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        ax3.set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        ax4.set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])

                    ax1.legend(loc='lower center')
                    ax2.legend(loc='lower center')
                    ax3.legend(loc='lower center')
                    ax4.legend(loc='lower center')

                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    custom_lines = [Line2D([0], [0], color=colors[0], lw=1.5),
                                    Line2D([0], [0], color=colors[1], lw=1.5, linestyle='--')]

                    fig.legend(custom_lines, ['Full Model', 'FMU'], loc='upper right')
                
                elif value["x-axis"]["Label"].lower() == "roll":
                    roll_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])
                    if value["x-axis"]["Unit"].lower() == "rad":
                        sweep = roll_sweep * 180 / np.pi
                    else:
                        sweep = roll_sweep

                    for roll_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self = self.roll(roll_val, roll_n_steps)

                        corner_vals[0].append(self.sus.state["FL_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        corner_vals[1].append(self.sus.state["FR_" + value["y-axis"]["Outputs"]["FR"]] * value["y-axis"]["Multipliers"]["FR"])
                        corner_vals[2].append(self.sus.state["RL_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                        corner_vals[3].append(self.sus.state["RR_" + value["y-axis"]["Outputs"]["RR"]] * value["y-axis"]["Multipliers"]["RR"])
                    
                    ax1.set_title(f"FL {value["Title"]}")
                    ax2.set_title(f"FR {value["Title"]}")
                    ax3.set_title(f"RL {value["Title"]}")
                    ax4.set_title(f"RR {value["Title"]}")

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    ax1.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    ax2.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    ax3.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    ax4.set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")

                    ax1.set_ylabel(f"FL {value["y-axis"]["Label"]} ({y_unit})")
                    ax2.set_ylabel(f"FR {value["y-axis"]["Label"]} ({y_unit})")
                    ax3.set_ylabel(f"RL {value["y-axis"]["Label"]} ({y_unit})")
                    ax4.set_ylabel(f"RR {value["y-axis"]["Label"]} ({y_unit})")

                    ax1.plot(roll_sweep, corner_vals[0])
                    ax2.plot(roll_sweep, corner_vals[1])
                    ax3.plot(roll_sweep, corner_vals[2])
                    ax4.plot(roll_sweep, corner_vals[3])

                    if self.FMU_config["Evaluate"]:
                        FL_FMU_vals = []
                        FR_FMU_vals = []
                        RL_FMU_vals = []
                        RR_FMU_vals = []

                        for roll in sweep:
                            FL_FMU_vals.append(FMU_fits["FL_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, 0, 0, roll]))[0])
                            FR_FMU_vals.append(FMU_fits["FR_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, 0, 0, roll]))[0])
                            RL_FMU_vals.append(FMU_fits["RL_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, 0, 0, roll]))[0])
                            RR_FMU_vals.append(FMU_fits["RR_" + value["y-axis"]["Outputs"]["FL"]](np.array([0, 0, 0, roll]))[0])

                        ax1.plot(roll_sweep, np.array(FL_FMU_vals) * value["y-axis"]["Multipliers"]["FL"], linestyle='--')
                        ax2.plot(roll_sweep, np.array(FR_FMU_vals) * value["y-axis"]["Multipliers"]["FR"], linestyle='--')
                        ax3.plot(roll_sweep, np.array(RL_FMU_vals) * value["y-axis"]["Multipliers"]["RL"], linestyle='--')
                        ax4.plot(roll_sweep, np.array(RR_FMU_vals) * value["y-axis"]["Multipliers"]["RR"], linestyle='--')

                    slope_0, func_0 = self.nom_tangent(a=corner_vals[0], b=roll_sweep)
                    slope_1, func_1 = self.nom_tangent(a=corner_vals[1], b=roll_sweep)
                    slope_2, func_2 = self.nom_tangent(a=corner_vals[2], b=roll_sweep)
                    slope_3, func_3 = self.nom_tangent(a=corner_vals[3], b=roll_sweep)

                    roll_range = max(roll_sweep) - min(roll_sweep)
                    tangent_bounds = [-0.10 * roll_range, 0.10 * roll_range]

                    ax1.plot(tangent_bounds, func_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax2.plot(tangent_bounds, func_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax3.plot(tangent_bounds, func_2(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_2, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    ax4.plot(tangent_bounds, func_3(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_3, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        ax1.set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        ax2.set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        ax3.set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        ax4.set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])

                    ax1.legend(loc='lower center')
                    ax2.legend(loc='lower center')
                    ax3.legend(loc='lower center')
                    ax4.legend(loc='lower center')

                    colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
                    custom_lines = [Line2D([0], [0], color=colors[0], lw=1.5),
                                    Line2D([0], [0], color=colors[1], lw=1.5, linestyle='--')]

                    fig.legend(custom_lines, ['Full Model', 'FMU'], loc='upper right')
                
                if value["Grid"]:
                    ax1.grid()
                    ax2.grid()
                    ax3.grid()
                    ax4.grid()
                
                if self.fit_config["Generate Linear"] or self.fit_config["Generate Cubic"]:
                    ax_table = fig.add_subplot(gs[:, 2])
                    ax_table.axis("off")

                    table_data = []
                    line1 = ax1.lines[0]
                    line2 = ax2.lines[0]
                    line3 = ax3.lines[0]
                    line4 = ax4.lines[0]

                    line1_x = line1.get_xdata()
                    line1_y = line1.get_ydata()
                    line2_x = line2.get_xdata()
                    line2_y = line2.get_ydata()
                    line3_x = line3.get_xdata()
                    line3_y = line3.get_ydata()
                    line4_x = line4.get_xdata()
                    line4_y = line4.get_ydata()

                    if self.fit_config["Generate Linear"]:
                        slope_0, func_0 = self.nom_tangent(a=line1_y, b=line1_x)
                        slope_1, func_1 = self.nom_tangent(a=line2_y, b=line2_x)
                        slope_2, func_2 = self.nom_tangent(a=line3_y, b=line3_x)
                        slope_3, func_3 = self.nom_tangent(a=line4_y, b=line4_x)

                        coeff_0_0 = slope_0
                        coeff_0_1 = round(float(func_0(0)), 3) if abs(float(func_0(0))) < 999 else format(float(func_0(0)), ".3e")

                        coeff_1_0 = slope_1
                        coeff_1_1 = round(float(func_1(0)), 3) if abs(float(func_1(0))) < 999 else format(float(func_1(0)), ".3e")

                        coeff_2_0 = slope_2
                        coeff_2_1 = round(float(func_2(0)), 3) if abs(float(func_2(0))) < 999 else format(float(func_2(0)), ".3e")

                        coeff_3_0 = slope_3
                        coeff_3_1 = round(float(func_3(0)), 3) if abs(float(func_3(0))) < 999 else format(float(func_3(0)), ".3e")

                        table_data.append(["Linear Fit", r"$f(x) = a_{1}x + a_{0}$"])
                        table_data.append(["FL", f"{r'$f(x) = $'}{round(coeff_0_0, 3)}{r'$x + $'}{coeff_0_1}"])
                        table_data.append(["FR", f"{r'$f(x) = $'}{round(coeff_1_0, 3)}{r'$x + $'}{coeff_1_1}"])
                        table_data.append(["RL", f"{r'$f(x) = $'}{round(coeff_2_0, 3)}{r'$x + $'}{coeff_2_1}"])
                        table_data.append(["RR", f"{r'$f(x) = $'}{round(coeff_3_0, 3)}{r'$x + $'}{coeff_3_1}"])
                    
                    if self.fit_config["Generate Cubic"]:
                        if self.fit_config["Generate Linear"]:
                            table_data.append([None, None])
                        
                        a_3, a_2, a_1, a_0 = np.polyfit(line1_x, line1_y, 3)
                        if not abs(a_0) < 999:
                            a_0 = format(a_0, ".1e")
                        else:
                            a_0 = round(a_0, 3)
                        
                        b_3, b_2, b_1, b_0 = np.polyfit(line2_x, line2_y, 3)
                        if not abs(b_0) < 999:
                            b_0 = format(b_0, ".1e")
                        else:
                            b_0 = round(b_0, 3)

                        c_3, c_2, c_1, c_0 = np.polyfit(line3_x, line3_y, 3)
                        if not abs(c_0) < 999:
                            c_0 = format(c_0, ".1e")
                        else:
                            c_0 = round(c_0, 3)

                        d_3, d_2, d_1, d_0 = np.polyfit(line4_x, line4_y, 3)
                        if not abs(d_0) < 999:
                            d_0 = format(d_0, ".1e")
                        else:
                            d_0 = round(d_0, 3)

                        table_data.append(["Cubic Fit", r"$f(x) = a_{3}x^{3} + a_{2}x^{2} + a_{1}x + a_{0}$"])
                        table_data.append(["FL", f"{r'$f(x) = $'}{round(a_3, 3)}{r'$x^{3} + $'}{round(a_2, 3)}{r'$x^{2} + $'}{round(a_1, 3)}{r'$x + $'}{a_0}"])
                        table_data.append(["FR", f"{r'$f(x) = $'}{round(b_3, 3)}{r'$x^{3} + $'}{round(b_2, 3)}{r'$x^{2} + $'}{round(b_1, 3)}{r'$x + $'}{b_0}"])
                        table_data.append(["RL", f"{r'$f(x) = $'}{round(c_3, 3)}{r'$x^{3} + $'}{round(c_2, 3)}{r'$x^{2} + $'}{round(c_1, 3)}{r'$x + $'}{c_0}"])
                        table_data.append(["RR", f"{r'$f(x) = $'}{round(d_3, 3)}{r'$x^{3} + $'}{round(d_2, 3)}{r'$x^{2} + $'}{round(d_1, 3)}{r'$x + $'}{d_0}"])

                    # Create table
                    table = ax_table.table(
                        cellText=table_data,
                        loc="center",
                        cellLoc="center",
                        colWidths=[0.2, 0.8]
                    )

                    table.scale(1, 2.2)
                    table.auto_set_font_size(False)
                    table.set_fontsize(8)

                    # Clean section headers
                    count = 0
                    for i, row in enumerate(table_data):
                        if row[0] == None:
                            table[i, 0].set_visible(False)
                            table[i, 1].set_visible(False)
                            continue

                        if (count % 5) == 0:
                            cell = table[i, 0]
                            cell.set_text_props(weight="bold", ha="center")
                            cell.visible_edges = "open"
                            cell.set_linewidth(0)
                            cell.set_facecolor("#f0f0f0")

                            cell = table[i, 1]
                            cell.set_text_props(weight="bold", ha="center")
                            cell.visible_edges = "open"
                            cell.set_linewidth(0)
                            cell.set_facecolor("#f0f0f0")
                        
                        count += 1
                    
                    fig.tight_layout()

            fig.tight_layout()

            plots.append(fig)

        self._generate_pdf(figs=plots)

    def nom_tangent(self, a: MutableSequence[float], b: MutableSequence[float]) -> Tuple[float, Callable]:
        """
        ### Nom Tangent

        Line tangent to x-y traces in the nominal condition (jounce=roll=0)

        Parameters
        ----------
        a : MutableSequence[float]
            Trace of desired y-values
        b : MutableSequence[float]
            Trace of desired x-values

        Returns
        -------
        Tuple[float, Callable]
            A tuple containing the slope and tangent line at x=0, respectively
        """
        kin_curve = CubicSpline(b, a)
        kin_deriv = kin_curve.derivative()

        return (float(kin_deriv(0)), lambda x: float(kin_deriv(0)) * np.array(x) + float(kin_curve(0)))

    @SISO_cache
    def heave(self, heave):
        self.sus = deepcopy(self.sus_copy)
        self.sus.heave(heave=heave)
    
    @SISO_cache
    def roll(self, roll, n_steps):
        self.sus = deepcopy(self.sus_copy)
        self.sus.roll(roll=roll, n_steps=n_steps)

    def get_git_username(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.email"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"
    
    def get_git_name(self):
        try:
            name = subprocess.check_output(
                ["git", "config", "user.name"], stderr=subprocess.DEVNULL
            ).decode().strip()
            return name if name else "Unknown"
        except Exception:
            return "Unknown"

    def _generate_pdf(self, figs: Sequence[Figure]) -> Figure:
        p = PdfPages("./simulations/kin/kin_outputs/kin_plots.pdf")

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()
