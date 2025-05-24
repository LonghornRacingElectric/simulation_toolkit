from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension
from _4_custom_libraries.cache import SISO_cache

from typing import Callable, MutableSequence, Sequence, Set, Tuple
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import numpy as np
import yaml


class Kinematics:
    def __init__(self, model_path: str):
        sus_data = SuspensionData(path=model_path)
        self.sus = Suspension(sus_data=sus_data)

        roll_n_steps = 5
        
        with open("./simulations/kin/kin_inputs/kin.yml") as f:
            try:
                self.plot_config : dict[str, dict[str, dict]] = yaml.safe_load(f)
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)
        
        self.outputs: Set[str] = set([key[key.index("_") + 1:] for key in self.sus.state.keys()])

        plots = []

        total_evals = sum([x["x-axis"]["Number Steps"] for x in self.plot_config.values()])
        eval_num = 0

        for _, value in self.plot_config.items():
            num_plots = [x[1] for x in value["Corners"].items()].count(True)
            if num_plots == 2:
                axle_vals = [[], []]
                fig, axes = plt.subplots(nrows=1, ncols=2)
                fig.set_size_inches(w=11, h=8.5)
                fig.suptitle(value["Title"], fontsize=18)

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
                    
                    axes[0].set_title(f"Fr {value["Title"]}", fontsize=14)
                    axes[1].set_title(f"Rr {value["Title"]}", fontsize=14)
                    
                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    axes[0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    axes[1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    axes[0].set_ylabel(f"Fr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    axes[1].set_ylabel(f"Rr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)

                    axes[0].plot(jounce_sweep, axle_vals[0])
                    axes[1].plot(jounce_sweep, axle_vals[1])

                    slope_0, roc_0 = self.nom_tangent(a=axle_vals[0], b=jounce_sweep)
                    slope_1, roc_1 = self.nom_tangent(a=axle_vals[1], b=jounce_sweep)

                    jounce_range = max(jounce_sweep) - min(jounce_sweep)
                    tangent_bounds = [-0.10 * jounce_range, 0.10 * jounce_range]

                    axes[0].plot(tangent_bounds, roc_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1].plot(tangent_bounds, roc_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        axes[0].set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        axes[1].set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])

                    axes[0].legend(loc='upper right')
                    axes[1].legend(loc='upper right')

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
                    
                    axes[0].set_title(f"Fr {value["Title"]}", fontsize=14)
                    axes[1].set_title(f"Rr {value["Title"]}", fontsize=14)

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]
                    
                    axes[0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    axes[1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    axes[0].set_ylabel(f"Fr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    axes[1].set_ylabel(f"Rr {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    
                    axes[0].plot(roll_sweep, axle_vals[0])
                    axes[1].plot(roll_sweep, axle_vals[1])
                    
                    slope_0, roc_0 = self.nom_tangent(a=axle_vals[0], b=roll_sweep)
                    slope_1, roc_1 = self.nom_tangent(a=axle_vals[1], b=roll_sweep)

                    roll_range = max(roll_sweep) - min(roll_sweep)
                    tangent_bounds = [-0.10 * roll_range, 0.10 * roll_range]

                    axes[0].plot(tangent_bounds, roc_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1].plot(tangent_bounds, roc_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        axes[0].set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        axes[1].set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])

                    axes[0].legend(loc='upper right')
                    axes[1].legend(loc='upper right')
                    
                if value["Grid"]:
                    axes[0].grid()
                    axes[1].grid()
                        
            if num_plots == 4:
                corner_vals = [[], [], [], []]
                fig, axes = plt.subplots(nrows=2, ncols=2)
                fig.set_size_inches(w=11, h=8.5)
                fig.suptitle(value["Title"], fontsize=18)

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
                    
                    axes[0, 0].set_title(f"FL {value["Title"]}", fontsize=14)
                    axes[0, 1].set_title(f"FR {value["Title"]}", fontsize=14)
                    axes[1, 0].set_title(f"RL {value["Title"]}", fontsize=14)
                    axes[1, 1].set_title(f"RR {value["Title"]}", fontsize=14)

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    axes[0, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    axes[0, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    axes[1, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)
                    axes[1, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})", fontsize=12)

                    axes[0, 0].set_ylabel(f"FL {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    axes[0, 1].set_ylabel(f"FR {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    axes[1, 0].set_ylabel(f"RL {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)
                    axes[1, 1].set_ylabel(f"RR {value["y-axis"]["Label"]} ({y_unit})", fontsize=12)

                    axes[0, 0].plot(jounce_sweep, corner_vals[0])
                    axes[0, 1].plot(jounce_sweep, corner_vals[1])
                    axes[1, 0].plot(jounce_sweep, corner_vals[2])
                    axes[1, 1].plot(jounce_sweep, corner_vals[3])

                    slope_0, roc_0 = self.nom_tangent(a=corner_vals[0], b=jounce_sweep)
                    slope_1, roc_1 = self.nom_tangent(a=corner_vals[1], b=jounce_sweep)
                    slope_2, roc_2 = self.nom_tangent(a=corner_vals[2], b=jounce_sweep)
                    slope_3, roc_3 = self.nom_tangent(a=corner_vals[3], b=jounce_sweep)

                    jounce_range = max(jounce_sweep) - min(jounce_sweep)
                    tangent_bounds = [-0.10 * jounce_range, 0.10 * jounce_range]

                    axes[0, 0].plot(tangent_bounds, roc_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[0, 1].plot(tangent_bounds, roc_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1, 0].plot(tangent_bounds, roc_2(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_2, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1, 1].plot(tangent_bounds, roc_3(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_3, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        axes[0, 0].set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        axes[0, 1].set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        axes[1, 0].set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        axes[1, 1].set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])

                    axes[0, 0].legend(loc='upper right')
                    axes[0, 1].legend(loc='upper right')
                    axes[1, 0].legend(loc='upper right')
                    axes[1, 1].legend(loc='upper right')
                
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
                    
                    axes[0, 0].set_title(f"FL {value["Title"]}")
                    axes[0, 1].set_title(f"FR {value["Title"]}")
                    axes[1, 0].set_title(f"RL {value["Title"]}")
                    axes[1, 1].set_title(f"RR {value["Title"]}")

                    x_unit = value["x-axis"]["Unit"]
                    y_unit = value["y-axis"]["Unit"]

                    axes[0, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    axes[0, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    axes[1, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")
                    axes[1, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({x_unit})")

                    axes[0, 0].set_ylabel(f"FL {value["y-axis"]["Label"]} ({y_unit})")
                    axes[0, 1].set_ylabel(f"FR {value["y-axis"]["Label"]} ({y_unit})")
                    axes[1, 0].set_ylabel(f"RL {value["y-axis"]["Label"]} ({y_unit})")
                    axes[1, 1].set_ylabel(f"RR {value["y-axis"]["Label"]} ({y_unit})")

                    axes[0, 0].plot(roll_sweep, corner_vals[0])
                    axes[0, 1].plot(roll_sweep, corner_vals[1])
                    axes[1, 0].plot(roll_sweep, corner_vals[2])
                    axes[1, 1].plot(roll_sweep, corner_vals[3])

                    slope_0, roc_0 = self.nom_tangent(a=corner_vals[0], b=roll_sweep)
                    slope_1, roc_1 = self.nom_tangent(a=corner_vals[1], b=roll_sweep)
                    slope_2, roc_2 = self.nom_tangent(a=corner_vals[2], b=roll_sweep)
                    slope_3, roc_3 = self.nom_tangent(a=corner_vals[3], b=roll_sweep)

                    roll_range = max(roll_sweep) - min(roll_sweep)
                    tangent_bounds = [-0.10 * roll_range, 0.10 * roll_range]

                    axes[0, 0].plot(tangent_bounds, roc_0(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_0, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[0, 1].plot(tangent_bounds, roc_1(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_1, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1, 0].plot(tangent_bounds, roc_2(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_2, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')
                    axes[1, 1].plot(tangent_bounds, roc_3(tangent_bounds), label=rf"$\left.\frac{{{"dy"}}}{{{"dx"}}}\right|_{{{"x=0"}}}$ = {round(slope_3, 3):.3f} $\frac{{{y_unit}}}{{{x_unit}}}$", c='r')

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        axes[0, 0].set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        axes[0, 1].set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        axes[1, 0].set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        axes[1, 1].set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])

                    axes[0, 0].legend(loc='upper right')
                    axes[0, 1].legend(loc='upper right')
                    axes[1, 0].legend(loc='upper right')
                    axes[1, 1].legend(loc='upper right')
                
                if value["Grid"]:
                    axes[0, 0].grid()
                    axes[0, 1].grid()
                    axes[1, 0].grid()
                    axes[1, 1].grid()

            fig.tight_layout()

            plots.append(fig)

        self._generate_pdf(figs=plots)

    def nom_tangent(self, a: MutableSequence[float], b: MutableSequence[float]) -> Tuple[float, Callable]:
        """
        ### Nom Tangent

        Line tangent to x-y traces in the nominal condition (jounce=0, roll=0)

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
        nom_index = list(abs(b)).index(min(abs(b)))
        offset = 1

        m = (a[nom_index + offset] - a[nom_index]) / (b[nom_index + offset] - b[nom_index])

        return (m, lambda x: m * (x - b[nom_index]) + a[nom_index])

    @SISO_cache
    def heave(self, heave):
        self.sus.heave(heave=None)
        self.sus.heave(heave=heave)
    
    @SISO_cache
    def roll(self, roll, n_steps):
        self.sus.heave(heave=None)
        self.sus.roll(roll=roll, n_steps=n_steps)

    def _generate_pdf(self, figs: Sequence[Figure]) -> Figure:
        p = PdfPages("./simulations/kin/kin_outputs/kin_plots.pdf")

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()
