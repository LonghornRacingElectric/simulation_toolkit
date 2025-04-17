from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension

from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from typing import Sequence, Set
import numpy as np
import yaml


class Kinematics:
    def __init__(self):
        sus_data = SuspensionData(path="./1_model_inputs/Nightwatch.yml")
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
                        self.sus.heave(heave=None)
                        self.sus.heave(heave=jounce_val)
                        
                        for _, val in value["y-axis"]["Outputs"].items():
                            if (val not in self.outputs) and not (val == None):
                                raise Exception(f"Invalid output specified in {value["Title"]} ({val}). Please use one of the following: {", ".join(self.outputs)}")
                            
                        axle_vals[0].append(self.sus.state["Fr_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        axle_vals[1].append(self.sus.state["Rr_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                    
                    axes[0].set_title(f"Fr {value["Title"]}", fontsize=14)
                    axes[1].set_title(f"Rr {value["Title"]}", fontsize=14)
                    
                    axes[0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)
                    axes[1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)

                    axes[0].set_ylabel(f"Fr {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    axes[1].set_ylabel(f"Rr {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    
                    axes[0].plot(jounce_sweep, axle_vals[0])
                    axes[1].plot(jounce_sweep, axle_vals[1])

                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        axes[0].set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        axes[1].set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])
                
                elif value["x-axis"]["Label"].lower() == "roll":
                    roll_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])

                    if value["x-axis"]["Unit"].lower() == "rad":
                        sweep = roll_sweep * 180 / np.pi
                    else:
                        sweep = roll_sweep

                    for roll_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self.sus.heave(heave=None)
                        self.sus.roll(roll=roll_val, n_steps=roll_n_steps)
                        
                        for _, val in value["y-axis"]["Outputs"].items():
                            if (val not in self.outputs) and not (val == None):
                                raise Exception(f"Invalid output specified in {value["Title"]} ({val}). Please use one of the following: {", ".join(self.outputs)}")
                            
                        axle_vals[0].append(self.sus.state["Fr_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        axle_vals[1].append(self.sus.state["Rr_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                    
                    axes[0].set_title(f"Fr {value["Title"]}", fontsize=14)
                    axes[1].set_title(f"Rr {value["Title"]}", fontsize=14)
                    
                    axes[0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)
                    axes[1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)

                    axes[0].set_ylabel(f"Fr {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    axes[1].set_ylabel(f"Rr {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    
                    axes[0].plot(roll_sweep, axle_vals[0])
                    axes[1].plot(roll_sweep, axle_vals[1])
                    
                    if max(axle_vals[0]) - min(axle_vals[0]) < 1e-3:
                        axes[0].set_ylim([-1e-3 + np.mean(axle_vals[0]), 1e-3 + np.mean(axle_vals[0])])
                    if max(axle_vals[1]) - min(axle_vals[1]) < 1e-3:
                        axes[1].set_ylim([-1e-3 + np.mean(axle_vals[1]), 1e-3 + np.mean(axle_vals[1])])

                if value["Grid"]:
                    axes[0].grid()
                    axes[1].grid()
                        
            elif num_plots == 4:
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
                        self.sus.heave(heave=None)
                        self.sus.heave(heave=jounce_val)
                        
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

                    axes[0, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)
                    axes[0, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)
                    axes[1, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)
                    axes[1, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})", fontsize=12)

                    axes[0, 0].set_ylabel(f"FL {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    axes[0, 1].set_ylabel(f"FR {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    axes[1, 0].set_ylabel(f"RL {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)
                    axes[1, 1].set_ylabel(f"RR {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})", fontsize=12)

                    axes[0, 0].plot(jounce_sweep, corner_vals[0])
                    axes[0, 1].plot(jounce_sweep, corner_vals[1])
                    axes[1, 0].plot(jounce_sweep, corner_vals[2])
                    axes[1, 1].plot(jounce_sweep, corner_vals[3])

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        axes[0, 0].set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        axes[0, 1].set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        axes[1, 0].set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        axes[1, 1].set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])
                
                elif value["x-axis"]["Label"].lower() == "roll":
                    roll_sweep = np.linspace(*value["x-axis"]["Values"], value["x-axis"]["Number Steps"])

                    if value["x-axis"]["Unit"].lower() == "rad":
                        sweep = roll_sweep * 180 / np.pi
                    else:
                        sweep = roll_sweep

                    for roll_val in sweep:
                        eval_num += 1
                        print(f"Percent Completion: {round(eval_num / total_evals * 100, 2)}%\t", end="\r")
                        self.sus.heave(heave=None)
                        self.sus.roll(roll=roll_val, n_steps=roll_n_steps)

                        corner_vals[0].append(self.sus.state["FL_" + value["y-axis"]["Outputs"]["FL"]] * value["y-axis"]["Multipliers"]["FL"])
                        corner_vals[1].append(self.sus.state["FR_" + value["y-axis"]["Outputs"]["FR"]] * value["y-axis"]["Multipliers"]["FR"])
                        corner_vals[2].append(self.sus.state["RL_" + value["y-axis"]["Outputs"]["RL"]] * value["y-axis"]["Multipliers"]["RL"])
                        corner_vals[3].append(self.sus.state["RR_" + value["y-axis"]["Outputs"]["RR"]] * value["y-axis"]["Multipliers"]["RR"])
                    
                    axes[0, 0].set_title(f"FL {value["Title"]}")
                    axes[0, 1].set_title(f"FR {value["Title"]}")
                    axes[1, 0].set_title(f"RL {value["Title"]}")
                    axes[1, 1].set_title(f"RR {value["Title"]}")

                    axes[0, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})")
                    axes[0, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})")
                    axes[1, 0].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})")
                    axes[1, 1].set_xlabel(f"{value["x-axis"]["Label"]} ({value["x-axis"]["Unit"]})")

                    axes[0, 0].set_ylabel(f"FL {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})")
                    axes[0, 1].set_ylabel(f"FR {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})")
                    axes[1, 0].set_ylabel(f"RL {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})")
                    axes[1, 1].set_ylabel(f"RR {value["y-axis"]["Label"]} ({value["y-axis"]["Unit"]})")

                    axes[0, 0].plot(roll_sweep, corner_vals[0])
                    axes[0, 1].plot(roll_sweep, corner_vals[1])
                    axes[1, 0].plot(roll_sweep, corner_vals[2])
                    axes[1, 1].plot(roll_sweep, corner_vals[3])

                    if max(corner_vals[0]) - min(corner_vals[0]) < 1e-3:
                        axes[0, 0].set_ylim([-1e-3 + np.mean(corner_vals[0]), 1e-3 + np.mean(corner_vals[0])])
                    if max(corner_vals[1]) - min(corner_vals[1]) < 1e-3:
                        axes[0, 1].set_ylim([-1e-3 + np.mean(corner_vals[1]), 1e-3 + np.mean(corner_vals[1])])
                    if max(corner_vals[2]) - min(corner_vals[2]) < 1e-3:
                        axes[1, 0].set_ylim([-1e-3 + np.mean(corner_vals[2]), 1e-3 + np.mean(corner_vals[2])])
                    if max(corner_vals[3]) - min(corner_vals[3]) < 1e-3:
                        axes[1, 1].set_ylim([-1e-3 + np.mean(corner_vals[3]), 1e-3 + np.mean(corner_vals[3])])
                
                if value["Grid"]:
                    axes[0, 0].grid()
                    axes[0, 1].grid()
                    axes[1, 0].grid()
                    axes[1, 1].grid()

            fig.tight_layout()

            plots.append(fig)

        self._generate_pdf(figs=plots)

    def _generate_pdf(self, figs: Sequence[Figure]) -> Figure:
        p = PdfPages("./simulations/kin/kin_outputs/kin_plots.pdf")

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()
