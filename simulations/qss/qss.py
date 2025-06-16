from simulations.qss._qss_helpers.ymd_cv import YMDConstantVelocity
from simulations.qss._qss_helpers.ymd_cr import YMDConstantRadius

from scipy.interpolate import interp1d
from matplotlib.lines import Line2D

import matplotlib.pyplot as plt
import numpy as np
import yaml


class QSS:
    def __init__(self, model_path: str):
        with open("./simulations/qss/qss_inputs/qss.yml") as f:
            try:
                self.qss_config: dict[str, dict[str, dict]] = yaml.safe_load(f)
                self.ymd_config: dict[str, dict] = self.qss_config.pop("Yaw Moment Settings")
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)
        
        if self.ymd_config["Generate CV"]:
            velocity_results = []
            for vel in self.ymd_config["Velocity Schedule"]:
                cv_ymd_model = YMDConstantVelocity(model_path=model_path,
                                                   velX=vel,
                                                   hwa=self.ymd_config["Handwheel Angle Sweep"],
                                                   beta=self.ymd_config["Sideslip Angle Sweep"],
                                                   refinement=self.ymd_config["Refinement"])
                
                cv_ymd_output = cv_ymd_model.run()
                velocity_results.append(cv_ymd_output)

            self.cv_ymd_metrics(velocity_results[0])
            self.plot_cv_ymd(velocity_results[0])
        
        if self.ymd_config["Generate CR"]:
            radius_results = []
            for radius in self.ymd_config["Radius Schedule"]:
                cr_ymd_model = YMDConstantRadius(model_path=model_path,
                                                 radius=radius,
                                                 hwa=self.ymd_config["Handwheel Angle Sweep"],
                                                 beta=self.ymd_config["Sideslip Angle Sweep"],
                                                 refinement=self.ymd_config["Refinement"])
                
                cr_ymd_output = cr_ymd_model.run()
                radius_results.append(cr_ymd_output)

            self.plot_cr_ymd(radius_results[0])
    
    def cv_ymd_metrics(self, result_dict: dict) -> dict:
        {
            "hwa": {"angle":  [],
                    "accX":   [],
                    "accY":   [],
                    "accYaw": [],
                    "heave":  [],
                    "theta":  [],
                    "phi":    []},
        "beta": {"angle":  [],
                    "accX":   [],
                    "accY":   [],
                    "accYaw": [],
                    "heave":  [],
                    "theta":  [],
                "phi":    []}}
        
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()

        # Trim accY calculation
        accY_sorted = accY_lst[np.argsort(accYaw_lst)]
        accYaw_sorted = accYaw_lst[np.argsort(accYaw_lst)]
        trim_index = list(abs(accYaw_sorted)).index(min(abs(accYaw_sorted)))
        
        # Metrics
        max_accY = max(accY_lst)
        trim_ay = abs(accY_sorted[trim_index].__float__())
        N_at_max_accY = accYaw_lst[list(accY_lst).index(max_accY)]
        max_N = max(accYaw_lst)

        

    def plot_cv_ymd(self, result_dict: dict):
        fig = plt.figure()
        fig.set_size_inches(w=14, h=8.5)
        ax = fig.gca()

        velX = result_dict["consts"]["velX"][0]
        ax.set_title(f"Constant Velocity: {velX}" + r" $m/s$ | Yaw Acceleration vs Lateral Acceleration")

        ax.set_xlabel(r"Lateral Acceleration $(G)$")
        ax.set_ylabel(r"Yaw Acceleration $(rad/s^{2})$")
        ax.axhline(c="gray", linewidth=0.5)
        ax.axvline(c="gray", linewidth=0.5)

        for i in range(len(result_dict["hwa"]["accY"])):
            ax.plot(result_dict["hwa"]["accY"][i], result_dict["hwa"]["accYaw"][i], c='b')
        
        for i in range(len(result_dict["beta"]["accY"])):
            ax.plot(result_dict["beta"]["accY"][i], result_dict["beta"]["accYaw"][i], c='r')

        custom_lines = [Line2D([0], [0], color='b', lw=1.5),
                        Line2D([0], [0], color='r', lw=1.5)]

        fig.legend(custom_lines, [r"Constant $\delta$ (hwa)", r"Constant $\beta$ (beta)"], loc='upper right')

        ax.legend()
        ax.grid()
        
        plt.show()
    
    def plot_cr_ymd(self, result_dict: dict):
        fig = plt.figure()
        fig.set_size_inches(w=14, h=8.5)
        ax = fig.gca()

        radius = result_dict["consts"]["radius"][0]
        ax.set_title(f"Constant Radius: {radius}" + r" $m$ | Yaw Acceleration vs Lateral Acceleration")

        ax.set_xlabel(r"Lateral Acceleration $(G)$")
        ax.set_ylabel(r"Yaw Acceleration $(rad/s^{2})$")
        ax.axhline(c="gray", linewidth=0.5)
        ax.axvline(c="gray", linewidth=0.5)

        for i in range(len(result_dict["hwa"]["accY"])):
            ax.plot(result_dict["hwa"]["accY"][i], result_dict["hwa"]["accYaw"][i], c='b')
        
        for i in range(len(result_dict["beta"]["accY"])):
            ax.plot(result_dict["beta"]["accY"][i], result_dict["beta"]["accYaw"][i], c='r')

        custom_lines = [Line2D([0], [0], color='b', lw=1.5),
                        Line2D([0], [0], color='r', lw=1.5)]

        fig.legend(custom_lines, [r"Constant $\delta$ (hwa)", r"Constant $\beta$ (beta)"], loc='upper right')

        ax.legend()
        ax.grid()
        
        plt.show()