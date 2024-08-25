from vehicle_model.vehicle_model import VehicleModel
from matplotlib.backends.backend_pdf import PdfPages
from scipy.optimize import minimize
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from typing import Sequence
import numpy as np


class KinOptimization:
    def __init__(self, vehicle_model: VehicleModel, steer_sweep: np.ndarray, heave_sweep: np.ndarray, pitch_sweep: np.ndarray, roll_sweep: np.ndarray) -> None:
        self.vehicle = vehicle_model
        self.suspension = self.vehicle.suspension
        self.cg = self.suspension.cg

        # Store double wishbones
        self.FL_dw = self.suspension.FL_double_wishbone
        self.FR_dw = self.suspension.FR_double_wishbone
        self.RL_dw = self.suspension.RL_double_wishbone
        self.RR_dw = self.suspension.RR_double_wishbone
        self.dws = [self.FL_dw, self.FR_dw, self.RL_dw, self.RR_dw]

        # Store geometric properties
        self.FL_cg_x = abs(self.FL_dw.contact_patch.position[0] - self.cg.position[0])
        self.FR_cg_x = abs(self.FR_dw.contact_patch.position[0] - self.cg.position[0])
        self.RL_cg_x = abs(self.RL_dw.contact_patch.position[0] - self.cg.position[0])
        self.RR_cg_x = abs(self.RR_dw.contact_patch.position[0] - self.cg.position[0])
        self.FL_cg_y = abs(self.FL_dw.contact_patch.position[1] - self.cg.position[1])
        self.FR_cg_y = abs(self.FR_dw.contact_patch.position[1] - self.cg.position[1])
        self.RL_cg_y = abs(self.RL_dw.contact_patch.position[1] - self.cg.position[1])
        self.RR_cg_y = abs(self.RR_dw.contact_patch.position[1] - self.cg.position[1])

        # Store sweeps
        self.steer_sweep = steer_sweep
        self.heave_sweep = heave_sweep
        self.pitch_sweep = pitch_sweep
        self.roll_sweep = roll_sweep

        # Store tires
        self.FL_tire = self.vehicle.FL_tire
        self.FR_tire = self.vehicle.FR_tire
        self.RL_tire = self.vehicle.RL_tire
        self.RR_tire = self.vehicle.RR_tire
    
    def optimize(self) -> None:
        self.FL_UFI = self.FL_dw.upper_fore_link.inboard_node
        self.FL_UAI = self.FL_dw.upper_aft_link.inboard_node
        self.FL_LFI = self.FL_dw.lower_fore_link.inboard_node
        self.FL_LAI = self.FL_dw.lower_aft_link.inboard_node
        self.FL_SI = self.FL_dw.steering_link.inboard_node
        self.FL_UFO = self.FL_dw.upper_fore_link.outboard_node
        self.FL_LFO = self.FL_dw.lower_fore_link.outboard_node
        self.FL_SO = self.FL_dw.steering_link.outboard_node

        self.FR_UFI = self.FR_dw.upper_fore_link.inboard_node
        self.FR_UAI = self.FR_dw.upper_aft_link.inboard_node
        self.FR_LFI = self.FR_dw.lower_fore_link.inboard_node
        self.FR_LAI = self.FR_dw.lower_aft_link.inboard_node
        self.FR_SI = self.FR_dw.steering_link.inboard_node
        self.FR_UFO = self.FR_dw.upper_fore_link.outboard_node
        self.FR_LFO = self.FR_dw.lower_fore_link.outboard_node
        self.FR_SO = self.FR_dw.steering_link.outboard_node

        self.RL_UFI = self.RL_dw.upper_fore_link.inboard_node
        self.RL_UAI = self.RL_dw.upper_aft_link.inboard_node
        self.RL_LFI = self.RL_dw.lower_fore_link.inboard_node
        self.RL_LAI = self.RL_dw.lower_aft_link.inboard_node
        self.RL_SI = self.RL_dw.steering_link.inboard_node
        self.RL_UFO = self.RL_dw.upper_fore_link.outboard_node
        self.RL_LFO = self.RL_dw.lower_fore_link.outboard_node
        self.RL_SO = self.RL_dw.steering_link.outboard_node

        self.RR_UFI = self.RR_dw.upper_fore_link.inboard_node
        self.RR_UAI = self.RR_dw.upper_aft_link.inboard_node
        self.RR_LFI = self.RR_dw.lower_fore_link.inboard_node
        self.RR_LAI = self.RR_dw.lower_aft_link.inboard_node
        self.RR_SI = self.RR_dw.steering_link.inboard_node
        self.RR_UFO = self.RR_dw.upper_fore_link.outboard_node
        self.RR_LFO = self.RR_dw.lower_fore_link.outboard_node
        self.RR_SO = self.RR_dw.steering_link.outboard_node

        initial_pos = [self.FL_UFI.position, self.FL_UAI.position, self.FL_LFI.position, self.FL_LAI.position, self.FL_SI.position, self.FL_UFO.position, self.FL_LFO.position, self.FL_SO.position, \
                       self.RL_UFI.position, self.RL_UAI.position, self.RL_LFI.position, self.RL_LAI.position, self.RL_SI.position, self.RL_UFO.position, self.RL_LFO.position, self.RL_SO.position]
        
        initial_pos = np.array(initial_pos).flatten()

        bounds = []

        for val in initial_pos:
            lower = val - 1 * 0.0254
            upper = val + 1 * 0.0254
            bounds.append([lower, upper])

        self.counter = 0
        hdpt_locations = minimize(self.resid_func, x0=np.array(initial_pos).flatten(), bounds=bounds).x
        print(hdpt_locations)

    def resid_func(self, x):
        print(x)
        print(f"Iterations Complete: {self.counter}")
        FL_node_lst = [self.FL_UFI.position, self.FL_UAI.position, self.FL_LFI.position, self.FL_LAI.position, self.FL_SI.position, self.FL_UFO.position, self.FL_LFO.position, self.FL_SO.position]
        FR_node_lst = [self.FR_UFI.position, self.FR_UAI.position, self.FR_LFI.position, self.FR_LAI.position, self.FR_SI.position, self.FR_UFO.position, self.FR_LFO.position, self.FR_SO.position]
        RL_node_lst = [self.RL_UFI.position, self.RL_UAI.position, self.RL_LFI.position, self.RL_LAI.position, self.RL_SI.position, self.RL_UFO.position, self.RL_LFO.position, self.RL_SO.position]
        RR_node_lst = [self.RR_UFI.position, self.RR_UAI.position, self.RR_LFI.position, self.RR_LAI.position, self.RR_SI.position, self.RR_UFO.position, self.RR_LFO.position, self.RR_SO.position]

        for i in range(int(len(x) / 6)):
            FL_node_lst[i] = [x[i * 3], x[i * 3 + 1], x[i * 3 + 2]]
            FR_node_lst[i] = [x[i * 3], -1 * x[i * 3 + 1], x[i * 3 + 2]]

        for i in range(int(len(x) / 6)):
            RL_node_lst[i] = [x[i * 3 + 24], x[i * 3 + 1 + 24], x[i * 3 + 2 + 24]]
            RR_node_lst[i] = [x[i * 3 + 24], -1 * x[i * 3 + 1 + 24], x[i * 3 + 2 + 24]]

        FL_gamma_outputs = []
        FR_gamma_outputs = []
        RL_gamma_outputs = []
        RR_gamma_outputs = []
        counter = 0
        total_count = len(self.heave_sweep)**3

        for heave in self.heave_sweep:
            for pitch in self.pitch_sweep:
                for roll in self.roll_sweep:
                    # print(f"Plotting Progress: {round(counter / total_count * 100, 2)}", end="\r")
                    FL_jounce = heave + (self.FL_cg_x * np.sin(pitch * np.pi / 180) + self.FL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    FR_jounce = heave + (self.FR_cg_x * np.sin(pitch * np.pi / 180) + self.FR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    RL_jounce = heave + (self.RL_cg_x * np.sin(pitch * np.pi / 180) + self.RL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    RR_jounce = heave + (self.RR_cg_x * np.sin(pitch * np.pi / 180) + self.RR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))

                    self.FL_dw.jounce(jounce=FL_jounce)
                    self.FR_dw.jounce(jounce=FR_jounce)
                    self.RL_dw.jounce(jounce=RL_jounce)
                    self.RR_dw.jounce(jounce=RR_jounce)

                    FL_gamma_outputs.append(self.FL_dw.inclination_angle * 180 / np.pi)
                    FR_gamma_outputs.append(self.FR_dw.inclination_angle * 180 / np.pi)
                    RL_gamma_outputs.append(self.RL_dw.inclination_angle * 180 / np.pi)
                    RR_gamma_outputs.append(self.RR_dw.inclination_angle * 180 / np.pi)

        Fy_lst = []

        alpha_sweep = np.linspace(-20, 20, 40)

        for i in range(len(FL_gamma_outputs)):
            FL_max_Fy = max(np.abs(self.FL_tire.tire_eval(FZ=self.FL_tire.get_FNOMIN(), alpha=alpha_sweep * np.pi / 180, kappa=0, gamma=FL_gamma_outputs[i])[1]))
            FR_max_Fy = max(np.abs(self.FR_tire.tire_eval(FZ=self.FR_tire.get_FNOMIN(), alpha=alpha_sweep * np.pi / 180, kappa=0, gamma=FR_gamma_outputs[i])[1]))
            RL_max_Fy = max(np.abs(self.RL_tire.tire_eval(FZ=self.RL_tire.get_FNOMIN(), alpha=alpha_sweep * np.pi / 180, kappa=0, gamma=RL_gamma_outputs[i])[1]))
            RR_max_Fy = max(np.abs(self.RR_tire.tire_eval(FZ=self.RR_tire.get_FNOMIN(), alpha=alpha_sweep * np.pi / 180, kappa=0, gamma=RR_gamma_outputs[i])[1]))

            Fy_lst.append(FL_max_Fy)
            Fy_lst.append(FR_max_Fy)
            Fy_lst.append(RL_max_Fy)
            Fy_lst.append(RR_max_Fy)

        self.counter += 1
        
        print(f"Current Residual: {1 / np.linalg.norm(Fy_lst) / 1e-10}")
        
        return 1 / np.linalg.norm(Fy_lst) / 1e-10
    
    def plot(self) -> None:
        # Full version
        FL_gamma_outputs = []
        FR_gamma_outputs = []
        RL_gamma_outputs = []
        RR_gamma_outputs = []

        FL_toe_outputs = []
        FR_toe_outputs = []
        RL_toe_outputs = []
        RR_toe_outputs = []

        jounces = []

        total_count = len(self.heave_sweep)**3
        counter = 0

        self.suspension.full_suspension.transform_origin = False

        # self.suspension.full_suspension.reset_position()
        # for dw in self.dws:
        #     dw.heave_jounce = 0
        #     dw.pitch_jounce = 0
        #     dw.roll_jounce = 0
        #     dw.induced_steer = 0
        
        counter = 0
        for heave in self.heave_sweep:
            for pitch in self.pitch_sweep:
                for roll in self.roll_sweep:
                    print(f"Plotting Progress: {round(counter / total_count * 100, 2)}", end="\r")
                    FL_jounce = heave + (self.FL_cg_x * np.sin(pitch * np.pi / 180) + self.FL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    FR_jounce = heave + (self.FR_cg_x * np.sin(pitch * np.pi / 180) + self.FR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    RL_jounce = heave + (self.RL_cg_x * np.sin(pitch * np.pi / 180) + self.RL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
                    RR_jounce = heave + (self.RR_cg_x * np.sin(pitch * np.pi / 180) + self.RR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))

                    self.FL_dw.jounce(jounce=FL_jounce)
                    self.FR_dw.jounce(jounce=FR_jounce)
                    self.RL_dw.jounce(jounce=RL_jounce)
                    self.RR_dw.jounce(jounce=RR_jounce)

                    FL_gamma_outputs.append(self.FL_dw.inclination_angle * 180 / np.pi)
                    FR_gamma_outputs.append(self.FR_dw.inclination_angle * 180 / np.pi)
                    RL_gamma_outputs.append(self.RL_dw.inclination_angle * 180 / np.pi)
                    RR_gamma_outputs.append(self.RR_dw.inclination_angle * 180 / np.pi)

                    FL_toe_outputs.append(self.FL_dw.toe * 180 / np.pi)
                    FR_toe_outputs.append(self.FR_dw.toe * 180 / np.pi)
                    RL_toe_outputs.append(self.RL_dw.toe * 180 / np.pi)
                    RR_toe_outputs.append(self.RR_dw.toe * 180 / np.pi)

                    jounces.append(self.suspension.FL_double_wishbone.total_jounce)

                    counter += 1

        # Gamma comparison
        fig_1, ax_1 = plt.subplots(nrows=2, ncols=4)
        fig_1.set_size_inches(w=11, h=8.5)

        fig_1.subplots_adjust(wspace=0.2, hspace=0.4)

        fig_1.suptitle("Kin Correlation | Gamma")
        
        # Histograms
        ax_1[0, 0].set_title("FL Gamma Histogram")
        ax_1[0, 0].set_xlabel("FL Gamma (deg)")
        ax_1[0, 0].set_ylabel("Frequency")
        ax_1[0, 0].hist(x=FL_gamma_outputs, alpha=0.5)

        ax_1[0, 1].set_title("FR Gamma Histogram")
        ax_1[0, 1].set_xlabel("FR Gamma (deg)")
        ax_1[0, 1].set_ylabel("Frequency")
        ax_1[0, 1].hist(x=FR_gamma_outputs, alpha=0.5)

        ax_1[0, 2].set_title("RL Gamma Histogram")
        ax_1[0, 2].set_xlabel("RL Gamma (deg)")
        ax_1[0, 2].set_ylabel("Frequency")
        ax_1[0, 2].hist(x=RL_gamma_outputs, alpha=0.5)

        ax_1[0, 3].set_title("RR Gamma Histogram")
        ax_1[0, 3].set_xlabel("RR Gamma (deg)")
        ax_1[0, 3].set_ylabel("Frequency")
        ax_1[0, 3].hist(x=RR_gamma_outputs, alpha=0.5)

        # CDFs
        ax_1[1, 0].set_title("FL Gamma CDF")
        ax_1[1, 0].set_xlabel("FL Gamma (deg)")
        ax_1[1, 0].set_ylabel("Frequency")
        ax_1[1, 0].ecdf(x=FL_gamma_outputs, alpha=0.5)

        ax_1[1, 1].set_title("FR Gamma CDF")
        ax_1[1, 1].set_xlabel("FR Gamma (deg)")
        ax_1[1, 1].set_ylabel("Frequency")
        ax_1[1, 1].ecdf(x=FR_gamma_outputs, alpha=0.5)

        ax_1[1, 2].set_title("RL Gamma CDF")
        ax_1[1, 2].set_xlabel("RL Gamma (deg)")
        ax_1[1, 2].set_ylabel("Frequency")
        ax_1[1, 2].ecdf(x=RL_gamma_outputs, alpha=0.5)

        ax_1[1, 3].set_title("RR Gamma CDF")
        ax_1[1, 3].set_xlabel("RR Gamma (deg)")
        ax_1[1, 3].set_ylabel("Frequency")
        ax_1[1, 3].ecdf(x=RL_gamma_outputs, alpha=0.5)
        fig_1.legend(["Lite Model"])


        fig_2, ax_2 = plt.subplots(nrows=2, ncols=4)
        fig_2.set_size_inches(w=11, h=8.5)

        fig_2.subplots_adjust(wspace=0.2, hspace=0.4)

        fig_2.suptitle("Kin Correlation | Toe")
        
        # Histograms
        ax_2[0, 0].set_title("FL toe Histogram")
        ax_2[0, 0].set_xlabel("FL toe (deg)")
        ax_2[0, 0].set_ylabel("Frequency")
        ax_2[0, 0].hist(x=FL_toe_outputs, alpha=0.5)

        ax_2[0, 1].set_title("FR toe Histogram")
        ax_2[0, 1].set_xlabel("FR toe (deg)")
        ax_2[0, 1].set_ylabel("Frequency")
        ax_2[0, 1].hist(x=FR_toe_outputs, alpha=0.5)

        ax_2[0, 2].set_title("RL toe Histogram")
        ax_2[0, 2].set_xlabel("RL toe (deg)")
        ax_2[0, 2].set_ylabel("Frequency")
        ax_2[0, 2].hist(x=RL_toe_outputs, alpha=0.5)

        ax_2[0, 3].set_title("RR toe Histogram")
        ax_2[0, 3].set_xlabel("RR toe (deg)")
        ax_2[0, 3].set_ylabel("Frequency")
        ax_2[0, 3].hist(x=RR_toe_outputs, alpha=0.5)

        # CDFs
        ax_2[1, 0].set_title("FL toe CDF")
        ax_2[1, 0].set_xlabel("FL toe (deg)")
        ax_2[1, 0].set_ylabel("Frequency")
        ax_2[1, 0].ecdf(x=FL_toe_outputs, alpha=0.5)

        ax_2[1, 1].set_title("FR toe CDF")
        ax_2[1, 1].set_xlabel("FR toe (deg)")
        ax_2[1, 1].set_ylabel("Frequency")
        ax_2[1, 1].ecdf(x=FR_toe_outputs, alpha=0.5)

        ax_2[1, 2].set_title("RL toe CDF")
        ax_2[1, 2].set_xlabel("RL toe (deg)")
        ax_2[1, 2].set_ylabel("Frequency")
        ax_2[1, 2].ecdf(x=RL_toe_outputs, alpha=0.5)

        ax_2[1, 3].set_title("RR toe CDF")
        ax_2[1, 3].set_xlabel("RR toe (deg)")
        ax_2[1, 3].set_ylabel("Frequency")
        ax_2[1, 3].ecdf(x=RL_toe_outputs, alpha=0.5)
        fig_2.legend(["Lite Model"])

        fig_3 = plt.figure()
        fig_3.set_size_inches(w=11, h=8.5)
        ax_3 = fig_3.gca()

        ax_3.hist(x=jounces, alpha=0.5)
        fig_3.legend(["Lite Model"])

        self.save_pdf(figs=[fig_1, fig_2, fig_3], save_path="./outputs/kin_correlation.pdf")

    def save_pdf(self, figs: Sequence[Figure], save_path: str) -> None:
        p = PdfPages(save_path)

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()

    # def plot(self) -> None:
    #     # Full version
    #     FL_gamma_outputs = []
    #     FR_gamma_outputs = []
    #     RL_gamma_outputs = []
    #     RR_gamma_outputs = []

    #     FL_toe_outputs = []
    #     FR_toe_outputs = []
    #     RL_toe_outputs = []
    #     RR_toe_outputs = []

    #     jounces = []

    #     # Simplified version
    #     FL_gamma_outputs_2 = []
    #     FR_gamma_outputs_2 = []
    #     RL_gamma_outputs_2 = []
    #     RR_gamma_outputs_2 = []

    #     FL_toe_outputs_2 = []
    #     FR_toe_outputs_2 = []
    #     RL_toe_outputs_2 = []
    #     RR_toe_outputs_2 = []

    #     jounces_2 = []

    #     total_count = len(self.heave_sweep)**3
    #     counter = 0

    #     self.suspension.full_suspension.transform_origin = True
    #     # self.suspension.full_suspension.transform_origin = False

    #     for heave in self.heave_sweep:
    #         self.suspension.heave(heave=heave)

    #         for pitch in self.pitch_sweep:
    #             self.suspension.pitch(pitch=pitch)

    #             for roll in self.roll_sweep:
    #                 print(f"Plotting Progress: {round(counter / total_count * 100, 2)}", end="\r")
    #                 self.suspension.roll(roll=roll)

    #                 FL_gamma_outputs.append(self.FL_dw.inclination_angle * 180 / np.pi)
    #                 FR_gamma_outputs.append(self.FR_dw.inclination_angle * 180 / np.pi)
    #                 RL_gamma_outputs.append(self.RL_dw.inclination_angle * 180 / np.pi)
    #                 RR_gamma_outputs.append(self.RR_dw.inclination_angle * 180 / np.pi)

    #                 FL_toe_outputs.append(self.FL_dw.toe * 180 / np.pi)
    #                 FR_toe_outputs.append(self.FR_dw.toe * 180 / np.pi)
    #                 RL_toe_outputs.append(self.RL_dw.toe * 180 / np.pi)
    #                 RR_toe_outputs.append(self.RR_dw.toe * 180 / np.pi)

    #                 jounces.append(self.suspension.FL_double_wishbone.total_jounce)
                    
    #                 counter += 1

    #     self.suspension.full_suspension.reset_position()
    #     for dw in self.dws:
    #         dw.heave_jounce = 0
    #         dw.pitch_jounce = 0
    #         dw.roll_jounce = 0
    #         dw.induced_steer = 0
        
    #     counter = 0
    #     for heave in self.heave_sweep:
    #         for pitch in self.pitch_sweep:
    #             for roll in self.roll_sweep:
    #                 print(f"Plotting Progress: {round(counter / total_count * 100, 2)}", end="\r")
    #                 FL_jounce = heave + (self.FL_cg_x * np.sin(pitch * np.pi / 180) + self.FL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
    #                 FR_jounce = heave + (self.FR_cg_x * np.sin(pitch * np.pi / 180) + self.FR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
    #                 RL_jounce = heave + (self.RL_cg_x * np.sin(pitch * np.pi / 180) + self.RL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
    #                 RR_jounce = heave + (self.RR_cg_x * np.sin(pitch * np.pi / 180) + self.RR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))

    #                 self.FL_dw.jounce(jounce=FL_jounce)
    #                 self.FR_dw.jounce(jounce=FR_jounce)
    #                 self.RL_dw.jounce(jounce=RL_jounce)
    #                 self.RR_dw.jounce(jounce=RR_jounce)

    #                 # FL_gamma_outputs_2.append(self.FL_dw.inclination_angle * 180 / np.pi + roll)
    #                 # FR_gamma_outputs_2.append(self.FR_dw.inclination_angle * 180 / np.pi + roll)
    #                 # RL_gamma_outputs_2.append(self.RL_dw.inclination_angle * 180 / np.pi + roll)
    #                 # RR_gamma_outputs_2.append(self.RR_dw.inclination_angle * 180 / np.pi + roll)
    #                 FL_gamma_outputs_2.append(self.FL_dw.inclination_angle * 180 / np.pi)
    #                 FR_gamma_outputs_2.append(self.FR_dw.inclination_angle * 180 / np.pi)
    #                 RL_gamma_outputs_2.append(self.RL_dw.inclination_angle * 180 / np.pi)
    #                 RR_gamma_outputs_2.append(self.RR_dw.inclination_angle * 180 / np.pi)

    #                 FL_toe_outputs_2.append(self.FL_dw.toe * 180 / np.pi)
    #                 FR_toe_outputs_2.append(self.FR_dw.toe * 180 / np.pi)
    #                 RL_toe_outputs_2.append(self.RL_dw.toe * 180 / np.pi)
    #                 RR_toe_outputs_2.append(self.RR_dw.toe * 180 / np.pi)

    #                 jounces_2.append(self.suspension.FL_double_wishbone.total_jounce)

    #                 counter += 1

    #     # Gamma comparison
    #     fig_1, ax_1 = plt.subplots(nrows=2, ncols=4)
    #     fig_1.set_size_inches(w=11, h=8.5)

    #     fig_1.subplots_adjust(wspace=0.2, hspace=0.4)

    #     fig_1.suptitle("Kin Correlation | Gamma")
        
    #     # Histograms
    #     ax_1[0, 0].set_title("FL Gamma Histogram")
    #     ax_1[0, 0].set_xlabel("FL Gamma (deg)")
    #     ax_1[0, 0].set_ylabel("Frequency")
    #     ax_1[0, 0].hist(x=FL_gamma_outputs, alpha=0.5)
    #     ax_1[0, 0].hist(x=FL_gamma_outputs_2, alpha=0.5)

    #     ax_1[0, 1].set_title("FR Gamma Histogram")
    #     ax_1[0, 1].set_xlabel("FR Gamma (deg)")
    #     ax_1[0, 1].set_ylabel("Frequency")
    #     ax_1[0, 1].hist(x=FR_gamma_outputs, alpha=0.5)
    #     ax_1[0, 1].hist(x=FR_gamma_outputs_2, alpha=0.5)

    #     ax_1[0, 2].set_title("RL Gamma Histogram")
    #     ax_1[0, 2].set_xlabel("RL Gamma (deg)")
    #     ax_1[0, 2].set_ylabel("Frequency")
    #     ax_1[0, 2].hist(x=RL_gamma_outputs, alpha=0.5)
    #     ax_1[0, 2].hist(x=RL_gamma_outputs_2, alpha=0.5)

    #     ax_1[0, 3].set_title("RR Gamma Histogram")
    #     ax_1[0, 3].set_xlabel("RR Gamma (deg)")
    #     ax_1[0, 3].set_ylabel("Frequency")
    #     ax_1[0, 3].hist(x=RR_gamma_outputs, alpha=0.5)
    #     ax_1[0, 3].hist(x=RR_gamma_outputs_2, alpha=0.5)

    #     # CDFs
    #     ax_1[1, 0].set_title("FL Gamma CDF")
    #     ax_1[1, 0].set_xlabel("FL Gamma (deg)")
    #     ax_1[1, 0].set_ylabel("Frequency")
    #     ax_1[1, 0].ecdf(x=FL_gamma_outputs, alpha=0.5)
    #     ax_1[1, 0].ecdf(x=FL_gamma_outputs_2, alpha=0.5)

    #     ax_1[1, 1].set_title("FR Gamma CDF")
    #     ax_1[1, 1].set_xlabel("FR Gamma (deg)")
    #     ax_1[1, 1].set_ylabel("Frequency")
    #     ax_1[1, 1].ecdf(x=FR_gamma_outputs, alpha=0.5)
    #     ax_1[1, 1].ecdf(x=FR_gamma_outputs_2, alpha=0.5)

    #     ax_1[1, 2].set_title("RL Gamma CDF")
    #     ax_1[1, 2].set_xlabel("RL Gamma (deg)")
    #     ax_1[1, 2].set_ylabel("Frequency")
    #     ax_1[1, 2].ecdf(x=RL_gamma_outputs, alpha=0.5)
    #     ax_1[1, 2].ecdf(x=RL_gamma_outputs_2, alpha=0.5)

    #     ax_1[1, 3].set_title("RR Gamma CDF")
    #     ax_1[1, 3].set_xlabel("RR Gamma (deg)")
    #     ax_1[1, 3].set_ylabel("Frequency")
    #     ax_1[1, 3].ecdf(x=RL_gamma_outputs, alpha=0.5)
    #     ax_1[1, 3].ecdf(x=RL_gamma_outputs_2, alpha=0.5)
    #     fig_1.legend(["Full Model", "Lite Model"])


    #     fig_2, ax_2 = plt.subplots(nrows=2, ncols=4)
    #     fig_2.set_size_inches(w=11, h=8.5)

    #     fig_2.subplots_adjust(wspace=0.2, hspace=0.4)

    #     fig_2.suptitle("Kin Correlation | Toe")
        
    #     # Histograms
    #     ax_2[0, 0].set_title("FL toe Histogram")
    #     ax_2[0, 0].set_xlabel("FL toe (deg)")
    #     ax_2[0, 0].set_ylabel("Frequency")
    #     ax_2[0, 0].hist(x=FL_toe_outputs, alpha=0.5)
    #     ax_2[0, 0].hist(x=FL_toe_outputs_2, alpha=0.5)

    #     ax_2[0, 1].set_title("FR toe Histogram")
    #     ax_2[0, 1].set_xlabel("FR toe (deg)")
    #     ax_2[0, 1].set_ylabel("Frequency")
    #     ax_2[0, 1].hist(x=FR_toe_outputs, alpha=0.5)
    #     ax_2[0, 1].hist(x=FR_toe_outputs_2, alpha=0.5)

    #     ax_2[0, 2].set_title("RL toe Histogram")
    #     ax_2[0, 2].set_xlabel("RL toe (deg)")
    #     ax_2[0, 2].set_ylabel("Frequency")
    #     ax_2[0, 2].hist(x=RL_toe_outputs, alpha=0.5)
    #     ax_2[0, 2].hist(x=RL_toe_outputs_2, alpha=0.5)

    #     ax_2[0, 3].set_title("RR toe Histogram")
    #     ax_2[0, 3].set_xlabel("RR toe (deg)")
    #     ax_2[0, 3].set_ylabel("Frequency")
    #     ax_2[0, 3].hist(x=RR_toe_outputs, alpha=0.5)
    #     ax_2[0, 3].hist(x=RR_toe_outputs_2, alpha=0.5)

    #     # CDFs
    #     ax_2[1, 0].set_title("FL toe CDF")
    #     ax_2[1, 0].set_xlabel("FL toe (deg)")
    #     ax_2[1, 0].set_ylabel("Frequency")
    #     ax_2[1, 0].ecdf(x=FL_toe_outputs, alpha=0.5)
    #     ax_2[1, 0].ecdf(x=FL_toe_outputs_2, alpha=0.5)

    #     ax_2[1, 1].set_title("FR toe CDF")
    #     ax_2[1, 1].set_xlabel("FR toe (deg)")
    #     ax_2[1, 1].set_ylabel("Frequency")
    #     ax_2[1, 1].ecdf(x=FR_toe_outputs, alpha=0.5)
    #     ax_2[1, 1].ecdf(x=FR_toe_outputs_2, alpha=0.5)

    #     ax_2[1, 2].set_title("RL toe CDF")
    #     ax_2[1, 2].set_xlabel("RL toe (deg)")
    #     ax_2[1, 2].set_ylabel("Frequency")
    #     ax_2[1, 2].ecdf(x=RL_toe_outputs, alpha=0.5)
    #     ax_2[1, 2].ecdf(x=RL_toe_outputs_2, alpha=0.5)

    #     ax_2[1, 3].set_title("RR toe CDF")
    #     ax_2[1, 3].set_xlabel("RR toe (deg)")
    #     ax_2[1, 3].set_ylabel("Frequency")
    #     ax_2[1, 3].ecdf(x=RL_toe_outputs, alpha=0.5)
    #     ax_2[1, 3].ecdf(x=RL_toe_outputs_2, alpha=0.5)
    #     fig_2.legend(["Full Model", "Lite Model"])

    #     fig_3 = plt.figure()
    #     fig_3.set_size_inches(w=11, h=8.5)
    #     ax_3 = fig_3.gca()

    #     ax_3.hist(x=jounces, alpha=0.5)
    #     ax_3.hist(x=jounces_2, alpha=0.5)
    #     fig_3.legend(["Full Model", "Lite Model"])

    #     self.save_pdf(figs=[fig_1, fig_2, fig_3], save_path="./outputs/kin_correlation.pdf")

    # def save_pdf(self, figs: Sequence[Figure], save_path: str) -> None:
    #     p = PdfPages(save_path)

    #     for page in figs:
    #         page.savefig(p, format="pdf")

    #     p.close()