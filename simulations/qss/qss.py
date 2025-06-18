from simulations.qss._qss_helpers.ymd_cv import YMDConstantVelocity
from simulations.qss._qss_helpers.ymd_cr import YMDConstantRadius
from _4_custom_libraries.simulation import Simulation

from scipy.interpolate import CubicSpline
import matplotlib.gridspec as gridspec
from scipy.spatial import ConvexHull
from matplotlib.lines import Line2D
from typing import Union, Tuple
from datetime import datetime
from PIL import Image

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
import tzlocal
import yaml


class QSS(Simulation):
    def __init__(self, model_path: str):
        with open("./simulations/qss/qss_inputs/qss.yml") as f:
            try:
                self.qss_config: dict[str, dict[str, dict]] = yaml.safe_load(f)
                self.ymd_config: dict[str, dict] = self.qss_config.pop("Yaw Moment Settings")
            except yaml.YAMLError as error:
                print("Failed to import yaml file. Reason:\n")
                print(error)
        
        cv_state_results = []
        self.public_cv_state_results = cv_state_results
        cv_ymd_metrics = []
        if self.ymd_config["Generate CV"]:
            for vel in self.ymd_config["Velocity Schedule"]:
                cv_ymd_model = YMDConstantVelocity(model_path=model_path,
                                                   velX=vel,
                                                   hwa=self.ymd_config["Handwheel Angle Sweep"],
                                                   beta=self.ymd_config["Sideslip Angle Sweep"],
                                                   refinement=self.ymd_config["Refinement"])
                
                cv_ymd_output = cv_ymd_model.run()
                ymd_metric_output = self.ymd_metrics(cv_ymd_output)
                
                cv_state_results.append(cv_ymd_output)
                cv_ymd_metrics.append(ymd_metric_output)

        cr_state_results = []
        self.public_cr_state_results = cr_state_results
        cr_ymd_metrics = []
        if self.ymd_config["Generate CR"]:
            for turn_radius in self.ymd_config["Radius Schedule"]:
                cr_ymd_model = YMDConstantRadius(model_path=model_path,
                                                 turn_radius=turn_radius,
                                                 hwa=self.ymd_config["Handwheel Angle Sweep"],
                                                 beta=self.ymd_config["Sideslip Angle Sweep"],
                                                 refinement=self.ymd_config["Refinement"])
                
                cr_ymd_output = cr_ymd_model.run()
                ymd_metric_output = self.ymd_metrics(cr_ymd_output)

                cr_state_results.append(cr_ymd_output)
                cr_ymd_metrics.append(ymd_metric_output)

        self.max_table_dict = {"max_accY":          "max" + r"$\left( a_{y} \right)$"                       ,
                               "max_trim_accY":     "max" + r"$\left( \left. a_{y} \right|_{N = 0} \right)$",
                               "N_at_max_accY":     r"$\left. N \right|_{max(a_{y})}$"                      ,
                            #    "alpha_at_max_accY": alpha_at_max_accY                                     ,
                               "beta_at_max_accY":  r"$\left. \beta \right|_{max(a_{y})}$"                  ,
                               "hwa_at_max_accY":   r"$\left. \delta \right|_{max(a_{y})}$"                 ,
                               "max_N":             "max" + r"$ \left( N \right)$"                          ,
                            #    "alpha_at_max_N":    alpha_at_max_N                                        ,
                               "beta_at_max_N":     r"$\left. \beta \right|_{max(N)}$"                      ,
                               "hwa_at_max_N":      r"$\left. \delta \right|_{max(N)}$"                     ,
                               "accY_at_max_N":     r"$\left. a_{y} \right|_{max(N)}$"                      ,
                               "dN_dd_b1_pos_accY": r"$\left. \frac{dN}{d \delta} \right|_{max(a_{y})}$"    ,
                               "dN_db_d1_pos_accY": r"$\left. \frac{dN}{d \beta} \right|_{max(a_{y})}$"     ,
                               "dN_dd_b0":          r"$\left. \frac{dN}{d \delta} \right|_{\beta = 0}$"     ,
                               "dN_db_d0":          r"$\left. \frac{dN}{d \beta} \right|_{\delta = 0}$"     ,
                               }

        self.min_table_dict = {"min_accY":          "min" + r"$\left( a_{y} \right)$"                       ,
                               "min_trim_accY":     "min" + r"$\left( \left. a_{y} \right|_{N = 0} \right)$",
                               "N_at_min_accY":     r"$\left. N \right|_{min(a_{y})}$"                      ,
                            #    "alpha_at_min_accY": alpha_at_min_accY                                     ,
                               "beta_at_min_accY":  r"$\left. \beta \right|_{min(a_{y})}$"                  ,
                               "hwa_at_min_accY":   r"$\left. \delta \right|_{min(a_{y})}$"                 ,
                               "min_N":             "min" + r"$ \left( N \right)$"                          ,
                            #    "alpha_at_min_N":    alpha_at_min_N                                        ,
                               "beta_at_min_N":     r"$\left. \beta \right|_{min(N)}$"                      ,
                               "hwa_at_min_N":      r"$\left. \delta \right|_{min(N)}$"                     ,
                               "accY_at_min_N":     r"$\left. a_{y} \right|_{min(N)}$"                      ,
                               "dN_dd_b1_neg_accY": r"$\left. \frac{dN}{d \delta} \right|_{min(a_{y})}$"    ,
                               "dN_db_d1_neg_accY": r"$\left. \frac{dN}{d \beta} \right|_{min(a_{y})}$"     ,
                               "dN_dd_b0":          r"$\left. \frac{dN}{d \delta} \right|_{\beta = 0}$"     ,
                               "dN_db_d0":          r"$\left. \frac{dN}{d \beta} \right|_{\delta = 0}$"     ,
                               }

        self.unit_lst = [r"$\left( m/s \right)$",
                         r"$\left( m/s \right)$",
                         r"$\left( Nm \right)$",
                        #  r"$\left( deg \right)$",
                         r"$\left( deg \right)$",
                         r"$\left( deg \right)$",
                         r"$\left( Nm \right)$",
                        #  r"$\left( deg \right)$",
                         r"$\left( deg \right)$",
                         r"$\left( deg \right)$",
                         r"$\left( m/s \right)$",
                         r"$\left( \frac{Nm}{deg} \right)$",
                         r"$\left( \frac{Nm}{deg} \right)$",
                         r"$\left( \frac{Nm}{deg} \right)$",
                         r"$\left( \frac{Nm}{deg} \right)$"]
        
        #########################
        ### REPORT GENERATION ###
        #########################
        plots = []
        ######### NOTES #########

        notes = []

        notes.append("The variable" + r" $\delta$ " + "always refers to the average heading angle of the front tires")
        
        #########################

        local_tz = tzlocal.get_localzone() # Get local time zone
        now = datetime.now(local_tz) # Get current local time with time zone

        title_fig = plt.figure(figsize=(14, 8.5), dpi=300)
            
        # Logo bullshit (this was all trial and error)
        logo_path = "_5_ico/lhrEnobackground.png"
        img = Image.open(logo_path) # Read image
        img_resized = img.resize((int(img.width * 0.35), int(img.height * 0.35)), Image.Resampling.LANCZOS) # Resize image
        img_np = mpimg.pil_to_array(img_resized) # Convert to array
        fig_height_px = title_fig.get_figheight() * title_fig.dpi # Get figure sizing
        title_fig.figimage(img_np, xo=10, yo=int(fig_height_px * 0.815), zorder=1) # Insert image

        # Insert text
        title_fig.text(0.5, 0.55, "Quasi-Steady-State Report", fontsize=30, ha='center')
        title_fig.text(0.5, 0.50, "Simulation Author: Robert Horvath", fontsize=12, ha='center')
        title_fig.text(0.5, 0.45, f"Generated By: {self.get_git_name()} ({self.get_git_username()})", fontsize=12, ha='center')
        title_fig.text(0.5, 0.40, f"Date: {now.strftime("%Y-%m-%d, %I:%M %p %Z")}", fontsize=12, ha='center')
        
        for index, note in enumerate(notes[::-1]):
            title_fig.text(0.02, 0.025 * index + 0.025, f"Note {(len(notes) - index)}: {note}", fontsize=8, ha='left')
        
        title_fig.gca().axis('off')
        
        plots.append(title_fig)

        #########################

        if len(self.ymd_config["Velocity Schedule"]) > 1:
            cv_metric_plots = self.cv_metric_plots(self.ymd_config["Velocity Schedule"], cv_ymd_metrics)
            for plot in cv_metric_plots:
                plots.append(plot)

        #########################

        appendix_fig = plt.figure(figsize=(14, 8.5), dpi=300)
        appendix_fig.text(0.5, 0.55, "Appendix", fontsize=30, ha='center')
        
        plots.append(appendix_fig)
        
        #########################

        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        axes = fig.subplots(nrows=2, ncols=3)
        metric_arrays = {}

        for metric_set in cv_ymd_metrics:
            for key in metric_set.keys():
                if key not in metric_arrays.keys():
                    metric_arrays[key] = [metric_set[key]]
                else:
                    metric_arrays[key] += [metric_set[key]]
        
        counter = 0
        for index, key in enumerate(metric_arrays.keys()):
            if key in self.max_table_dict.keys():
                axes[counter // 3, counter % 3].set_xlabel(r"Velocity $(m/s)$")
                axes[counter // 3, counter % 3].set_ylabel(self.max_table_dict[key])
                axes[counter // 3, counter % 3].set_title(self.max_table_dict[key] + " vs " + r"Velocity $(m/s)$")
                axes[counter // 3, counter % 3].plot(self.ymd_config["Velocity Schedule"], metric_arrays[key])
                axes[counter // 3, counter % 3].grid()
            else:
                continue

            counter += 1

            if counter == 6:
                fig.tight_layout(pad=2.5)
                plots.append(fig)

                fig = plt.figure(figsize=(14, 8.5), dpi=300)
                axes = fig.subplots(nrows=2, ncols=3)
                counter = 0
        
        if not counter == 0:
            for i in range(counter, 6):
                axes[i // 3, i % 3].axis("off")

            fig.tight_layout(pad=2.5)
            plots.append(fig)
        
        for cv_states, cv_metrics in zip(cv_state_results, cv_ymd_metrics):
            plots.append(self.plot_cv_ymd(cv_states=cv_states, cv_metrics=cv_metrics))
        
        for cr_states, cr_metrics in zip(cr_state_results, cr_ymd_metrics):
            plots.append(self.plot_cr_ymd(cr_states=cr_states, cr_metrics=cr_metrics))

        self._generate_pdf(figs=plots, save_path="./simulations/qss/qss_outputs/qss_report.pdf")
    
    def cv_metric_plots(self, velocity_schedule, ymd_metrics: list[dict]):
        cv_plots = []

        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(2, 3, height_ratios=[1, 1], width_ratios=[1, 1, 1])
        
        fig.text(0.015, 0.94, "Acceleration vs Velocity", fontsize=24, ha='left')

        peak_accY = []
        peak_trim_accY = []
        N_at_peak_accY = []

        for i in range(len(velocity_schedule)):
            avg_accY = (-1 * ymd_metrics[i]["min_accY"] + ymd_metrics[i]["max_accY"]) / 2
            avg_trim_accY = (-1 * ymd_metrics[i]["min_trim_accY"] + ymd_metrics[i]["max_trim_accY"]) / 2
            avg_N_at_peak_accY = (-1 * ymd_metrics[i]["N_at_min_accY"] + ymd_metrics[i]["N_at_max_accY"]) / 2
            
            peak_accY.append(avg_accY)
            peak_trim_accY.append(avg_trim_accY)
            N_at_peak_accY.append(avg_N_at_peak_accY)

        max_vel = max(velocity_schedule)
        min_vel = min(velocity_schedule)
        avg_vel = np.average(velocity_schedule)

        graph_ax = fig.add_subplot(gs[0, 0])
        peak_accY_deriv = CubicSpline(velocity_schedule, peak_accY).derivative()
        graph_ax.set_xlabel(r"Velocity $(m/s)$")
        graph_ax.set_ylabel(r"$A_{y}$ $(G)$")
        graph_ax.set_title(r"Peak $A_{y}$ vs Velocity")
        graph_ax.plot(velocity_schedule, [x / 9.81 for x in peak_accY])
        graph_ax.grid()

        graph_ax = fig.add_subplot(gs[0, 1])
        peak_trim_accY_deriv = CubicSpline(velocity_schedule, peak_trim_accY).derivative()
        graph_ax.set_xlabel(r"Velocity $(m/s)$")
        graph_ax.set_ylabel(r"$A_{y}$ $(G)$")
        graph_ax.set_title(r"Peak Trim $A_{y}$ vs Velocity")
        graph_ax.plot(velocity_schedule, [x / 9.81 for x in peak_trim_accY])
        graph_ax.grid()

        graph_ax = fig.add_subplot(gs[0, 2])
        N_at_peak_accY_deriv = CubicSpline(velocity_schedule, N_at_peak_accY).derivative()
        graph_ax.set_xlabel(r"Velocity $(m/s)$")
        graph_ax.set_ylabel(r"N $(Nm)$")
        graph_ax.set_title(r"Yaw Moment at Peak $A_{y}$ vs Velocity")
        graph_ax.plot(velocity_schedule, N_at_peak_accY)
        graph_ax.grid()
        
        # Tables!
        table_ax = fig.add_subplot(gs[1, 0])
        table_ax.axis("off")
        table_content = [[None, r"$\frac{d A_{y}}{d V} \quad \left( \frac{G}{m/s} \right)$"],
                         ["at Min Velocity", "{:.3f}".format(peak_accY_deriv(min_vel) / 9.81)],
                         ["at Avg Velocity", "{:.3f}".format(peak_accY_deriv(avg_vel) / 9.81)],
                         ["at Max Velocity", "{:.3f}".format(peak_accY_deriv(max_vel) / 9.81)]]

        metric_table = table_ax.table(
            cellText=table_content,
            loc="center",
            cellLoc="center",
            colWidths=[0.5, 0.5]
        )
        metric_table.scale(1, 5)
        metric_table.auto_set_font_size(False)
        metric_table.set_fontsize(14)

        for i in range(len(table_content)):
            for j in range(2):
                cell = metric_table[i, j]
                cell.visible_edges = "TB"
                cell.set_linewidth(1)
                cell.set_facecolor("#f0f0f0")
                cell.set_text_props(ha="center")
                
        for j in range(2):
            cell = metric_table[1, j]
            cell.visible_edges = "B"
            cell.set_text_props(ha="center") # weight="bold", 

        table_ax = fig.add_subplot(gs[1, 1])
        table_ax.axis("off")
        table_content = [[None, r"$\frac{d A_{y}}{d V} \quad \left( \frac{G}{m/s} \right)$"],
                         ["at Min Velocity", "{:.3f}".format(peak_trim_accY_deriv(min_vel) / 9.81)],
                         ["at Avg Velocity", "{:.3f}".format(peak_trim_accY_deriv(avg_vel) / 9.81)],
                         ["at Max Velocity", "{:.3f}".format(peak_trim_accY_deriv(max_vel) / 9.81)]]

        metric_table = table_ax.table(
            cellText=table_content,
            loc="center",
            cellLoc="center",
            colWidths=[0.5, 0.5]
        )
        metric_table.scale(1, 5)
        metric_table.auto_set_font_size(False)
        metric_table.set_fontsize(14)

        for i in range(len(table_content)):
            for j in range(2):
                cell = metric_table[i, j]
                cell.visible_edges = "TB"
                cell.set_linewidth(1)
                cell.set_facecolor("#f0f0f0")
                cell.set_text_props(ha="center")
                
        for j in range(2):
            cell = metric_table[1, j]
            cell.visible_edges = "B"
            cell.set_text_props(ha="center") # weight="bold", 
        
        table_ax = fig.add_subplot(gs[1, 2])
        table_ax.axis("off")
        table_content = [[None, r"$\frac{d N}{d V} \quad \left( \frac{Nm}{m/s} \right)$"],
                         ["at Min Velocity", "{:.3f}".format(N_at_peak_accY_deriv(min_vel) / 9.81)],
                         ["at Avg Velocity", "{:.3f}".format(N_at_peak_accY_deriv(avg_vel) / 9.81)],
                         ["at Max Velocity", "{:.3f}".format(N_at_peak_accY_deriv(max_vel) / 9.81)]]

        metric_table = table_ax.table(
            cellText=table_content,
            loc="center",
            cellLoc="center",
            colWidths=[0.5, 0.5]
        )
        metric_table.scale(1, 5)
        metric_table.auto_set_font_size(False)
        metric_table.set_fontsize(14)

        for i in range(len(table_content)):
            for j in range(2):
                cell = metric_table[i, j]
                cell.visible_edges = "TB"
                cell.set_linewidth(1)
                cell.set_facecolor("#f0f0f0")
                cell.set_text_props(ha="center")
                
        for j in range(2):
            cell = metric_table[1, j]
            cell.visible_edges = "B"
            cell.set_text_props(ha="center") # weight="bold", 

        fig.tight_layout()
        fig.subplots_adjust(left=0.075, right=0.95, top=0.85, bottom=0.10, wspace=0.35, hspace=0.35)
        cv_plots.append(fig)

        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(2, 2, height_ratios=[1, 1], width_ratios=[1, 1])
        
        fig.text(0.015, 0.94, "Control, Stability, and Handling", fontsize=24, ha='left')

        dN_dd_b0 = []
        dN_dd_b1 = []
        dN_db_d0 = []
        dN_db_d1 = []

        for i in range(len(velocity_schedule)):
            dN_dd_b0_val = ymd_metrics[i]["dN_dd_b0"]
            dN_dd_b1_val = (-1 * ymd_metrics[i]["dN_dd_b1_neg_accY"] + ymd_metrics[i]["dN_dd_b1_pos_accY"]) / 2
            dN_db_d0_val = ymd_metrics[i]["dN_db_d0"]
            dN_db_d1_val = (-1 * ymd_metrics[i]["dN_db_d1_neg_accY"] + ymd_metrics[i]["dN_db_d1_pos_accY"]) / 2
            
            dN_dd_b0.append(dN_dd_b0_val)
            dN_dd_b1.append(dN_dd_b1_val)
            dN_db_d0.append(dN_db_d0_val)
            dN_db_d1.append(dN_db_d1_val)
        
        graph_ax = fig.add_subplot(gs[0, 0])
        graph_ax.set_xlabel(r"Velocity $(m/s)$")
        graph_ax.set_ylabel(r"$\frac{dN}{d \delta} \quad \left( \frac{Nm}{deg} \right)$")
        graph_ax.set_title(r"Control Derivative vs Velocity")
        graph_ax.plot(velocity_schedule, dN_dd_b0, label=r'$\beta = 0$')
        graph_ax.plot(velocity_schedule, dN_dd_b1, label=r'$\left. \beta \right|_{max(A_{y})}$')
        graph_ax.legend(loc='upper right')
        graph_ax.grid()

        graph_ax = fig.add_subplot(gs[1, 0])
        graph_ax.set_xlabel(r"Velocity $(m/s)$")
        graph_ax.set_ylabel(r"$\frac{dN}{d \beta} \quad \left( \frac{Nm}{deg} \right)$")
        graph_ax.set_title(r"Stability Derivative vs Velocity")
        graph_ax.plot(velocity_schedule, dN_db_d0, label=r'$\delta = 0$')
        graph_ax.plot(velocity_schedule, dN_db_d1, label=r'$\left. \delta \right|_{max(A_{y})}$')
        graph_ax.legend(loc='upper right')
        graph_ax.grid()

        graph_ax = fig.add_subplot(gs[0, 1])
        graph_ax.set_xlabel(r"$A_{y} \quad (G)$")
        graph_ax.set_ylabel(r"$\delta \quad (deg)$")
        graph_ax.set_title(r"$\delta$ vs $A_{y}$")

        # Standard test is done with 100m radius
        radius_schedule = [x["consts"]["turn_radius"] for x in self.public_cr_state_results]
        target_radius_optim = abs(np.array(radius_schedule) - 100)
        target_radius_index = list(np.array(radius_schedule) - 100).index(target_radius_optim)
        target_result = self.public_cr_state_results[target_radius_index]

        cr_hwa_isolines = target_result["hwa"]
        cr_beta_isolines = target_result["beta"]
        trim_delta = []
        trim_accY = []

        for index, accYaws in enumerate(cr_hwa_isolines["accYaw"]):
            accYs = cr_hwa_isolines["accY"][index]
            deltas = [(x[0] + x[1]) / 2 for x in cr_hwa_isolines["delta"][index]]
            
            min_accYaw = min(abs(np.array(accYaws)))
            min_accYaw_index = list(abs(np.array(accYaws))).index(min_accYaw)

            if min_accYaw > 0.05 * max(abs(np.array(accYaws))):
                continue

            trim_delta.append(deltas[min_accYaw_index])
            trim_accY.append(accYs[min_accYaw_index])
        
        for index, accYaws in enumerate(cr_beta_isolines["accYaw"]):
            accYs = cr_beta_isolines["accY"][index]
            deltas = [(x[0] + x[1]) / 2 for x in cr_beta_isolines["delta"][index]]
            
            min_accYaw = min(abs(np.array(accYaws)))
            min_accYaw_index = list(abs(np.array(accYaws))).index(min_accYaw)
        
            trim_delta.append(deltas[min_accYaw_index])
            trim_accY.append(accYs[min_accYaw_index])

        sorted_trim_accY = sorted(trim_accY)
        sorted_trim_delta = [x for _, x in sorted(zip(trim_accY, trim_delta))]

        trim_accY_filtered = [x for x in sorted_trim_accY if x > 0]
        trim_delta_filtered = [delta for accY, delta in zip(sorted_trim_accY, sorted_trim_delta) if accY > 0]

        trim_accY_cleaned = []
        trim_delta_cleaned = []
        for index, accY in enumerate(trim_accY_filtered):
            if trim_accY_filtered[index:].count(accY) > 1:
                continue
            else:
                trim_accY_cleaned.append(accY)
                trim_delta_cleaned.append(trim_delta_filtered[index])

        graph_ax.plot(np.array(trim_accY_cleaned) / 9.81, trim_delta_cleaned)
        graph_ax.grid()

        graph_ax = fig.add_subplot(gs[1, 1])
        graph_ax.set_xlabel(r"$A_{y} \quad (G)$")
        graph_ax.set_ylabel(r"$K \quad \left( \frac{deg}{G} \right)$")
        graph_ax.set_title(r"Understeer Gradient vs $A_{y}$")
        
        numerical_deriv = np.diff(trim_delta_cleaned) / np.diff(np.array(trim_accY_cleaned) / 9.81)
        filtered_deriv = []
        filtered_accY = []
        for index, deriv in enumerate(numerical_deriv):
            if abs(deriv) > 10:
                continue
            else:
                filtered_deriv.append(deriv)
                filtered_accY.append(trim_accY_cleaned[index])

        graph_ax.plot(np.array(filtered_accY) / 9.81, filtered_deriv)
        graph_ax.grid()
        fig.subplots_adjust(left=0.10, right=0.925, top=0.85, bottom=0.125, wspace=0.20, hspace=0.35)

        cv_plots.append(fig)

        return cv_plots
    
    def ymd_metrics(self, result_dict: dict[str, dict[str, list[Union[float, list[float]]]]]) -> dict:
        # Metrics
        min_accY,          max_accY          = self.peak_accY_calc(          result_dict=result_dict)
        min_trim_accY,     max_trim_accY     = self.peak_trim_accY_calc(     result_dict=result_dict)
        N_at_min_accY,     N_at_max_accY     = self.N_at_peak_accY(          result_dict=result_dict)
        alpha_at_min_accY, alpha_at_max_accY = self.alpha_at_peak_accY_calc( result_dict=result_dict)
        beta_at_min_accY,  beta_at_max_accY  = self.beta_at_peak_accY_calc(  result_dict=result_dict)
        hwa_at_min_accY,   hwa_at_max_accY   = self.hwa_at_peak_accY_calc(   result_dict=result_dict)
        min_N,             max_N             = self.peak_N_calc(             result_dict=result_dict)
        alpha_at_min_N,    alpha_at_max_N    = self.alpha_at_peak_N_calc(    result_dict=result_dict)
        beta_at_min_N,     beta_at_max_N     = self.beta_at_peak_N_calc(     result_dict=result_dict)
        hwa_at_min_N,      hwa_at_max_N      = self.hwa_at_peak_N_calc(      result_dict=result_dict)
        accY_at_min_N,     accY_at_max_N     = self.accY_at_peak_N_calc(     result_dict=result_dict)
        
        dN_dd_b1_neg_accY, dN_dd_b1_pos_accY = self.dN_dd_b1_calc(           result_dict=result_dict)
        dN_db_d1_neg_accY, dN_db_d1_pos_accY = self.dN_db_d1_calc(           result_dict=result_dict)
        dN_dd_b0                             = self.dN_dd_b0_calc(           result_dict=result_dict)
        dN_db_d0                             = self.dN_db_d0_calc(           result_dict=result_dict)

        metric_dict = {"min_accY":          min_accY         ,
                       "max_accY":          max_accY         ,
                       "min_trim_accY":     min_trim_accY    ,
                       "max_trim_accY":     max_trim_accY    ,
                       "N_at_min_accY":     N_at_min_accY    ,
                       "N_at_max_accY":     N_at_max_accY    ,
                       "alpha_at_min_accY": alpha_at_min_accY,
                       "alpha_at_max_accY": alpha_at_max_accY,
                       "beta_at_min_accY":  beta_at_min_accY ,
                       "beta_at_max_accY":  beta_at_max_accY ,
                       "hwa_at_min_accY":   hwa_at_min_accY  ,
                       "hwa_at_max_accY":   hwa_at_max_accY  ,
                       "min_N":             min_N            ,
                       "max_N":             max_N            ,
                       "alpha_at_min_N":    alpha_at_min_N   ,
                       "alpha_at_max_N":    alpha_at_max_N   ,
                       "beta_at_min_N":     beta_at_min_N    ,
                       "beta_at_max_N":     beta_at_max_N    ,
                       "hwa_at_min_N":      hwa_at_min_N     ,
                       "hwa_at_max_N":      hwa_at_max_N     ,
                       "accY_at_min_N":     accY_at_min_N    ,
                       "accY_at_max_N":     accY_at_max_N    ,
                       "dN_dd_b1_neg_accY": dN_dd_b1_neg_accY,
                       "dN_dd_b1_pos_accY": dN_dd_b1_pos_accY,
                       "dN_db_d1_neg_accY": dN_db_d1_neg_accY,
                       "dN_db_d1_pos_accY": dN_db_d1_pos_accY,
                       "dN_dd_b0":          dN_dd_b0         ,
                       "dN_db_d0":          dN_db_d0         ,}
        
        return metric_dict
    
    def peak_accY_calc(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()

        return (min(accY_lst), max(accY_lst))

    def peak_trim_accY_calc(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()
        pts = np.array(list(zip(accY_lst, accYaw_lst)))

        cvx_hull = ConvexHull(pts)
        cvx_hull_points = pts[cvx_hull.vertices]

        intersections = []
        for i in range(len(cvx_hull_points)):
            p1 = cvx_hull_points[i]
            p2 = cvx_hull_points[(i + 1) % len(cvx_hull_points)]  # next point, wrap around

            y1, y2 = p1[1], p2[1]
            if y1 * y2 < 0:  # edge crosses the x-axis
                # Linear interpolation to find x where y = 0
                t = -y1 / (y2 - y1)
                x_intersect = p1[0] + t * (p2[0] - p1[0])
                intersections.append(x_intersect)

        return (min(intersections), max(intersections))

    def N_at_peak_accY(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)

        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)

        return (accYaw_lst[min_accY_index], accYaw_lst[max_accY_index])

    def alpha_at_peak_accY_calc(self, result_dict) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float, float]]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        alpha_lst = [item for sublist in result_dict["hwa"]["alpha"] + result_dict["beta"]["alpha"] for item in sublist]

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)

        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)

        return (alpha_lst[min_accY_index], alpha_lst[max_accY_index])
    
    def beta_at_peak_accY_calc(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        beta_lst = np.array(result_dict["hwa"]["beta"] + result_dict["beta"]["beta"]).flatten()

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)

        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)

        return (beta_lst[min_accY_index], beta_lst[max_accY_index])

    def hwa_at_peak_accY_calc(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()
        hwa_lst = np.array(result_dict["hwa"]["hwa"] + result_dict["beta"]["hwa"]).flatten()

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)

        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)

        return (hwa_lst[min_accY_index], hwa_lst[max_accY_index])

    def peak_N_calc(self, result_dict) -> Tuple[float, float]:
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()

        return (min(accYaw_lst), max(accYaw_lst))
    
    def alpha_at_peak_N_calc(self, result_dict) -> Tuple[Tuple[float, float, float, float], Tuple[float, float, float, float]]:
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()
        alpha_lst = [item for sublist in result_dict["hwa"]["alpha"] + result_dict["beta"]["alpha"] for item in sublist]

        min_accYaw = min(accYaw_lst)
        max_accYaw = max(accYaw_lst)

        min_accYaw_index = list(accYaw_lst).index(min_accYaw)
        max_accYaw_index = list(accYaw_lst).index(max_accYaw)

        return (alpha_lst[min_accYaw_index], alpha_lst[max_accYaw_index])
    
    def beta_at_peak_N_calc(self, result_dict) -> Tuple[float, float]:
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()
        beta_lst = np.array(result_dict["hwa"]["beta"] + result_dict["beta"]["beta"]).flatten()

        min_accYaw = min(accYaw_lst)
        max_accYaw = max(accYaw_lst)

        min_accYaw_index = list(accYaw_lst).index(min_accYaw)
        max_accYaw_index = list(accYaw_lst).index(max_accYaw)

        return (beta_lst[min_accYaw_index], beta_lst[max_accYaw_index])

    def hwa_at_peak_N_calc(self, result_dict) -> Tuple[float, float]:
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()
        hwa_lst = np.array(result_dict["hwa"]["hwa"] + result_dict["beta"]["hwa"]).flatten()

        min_accYaw = min(accYaw_lst)
        max_accYaw = max(accYaw_lst)

        min_accYaw_index = list(accYaw_lst).index(min_accYaw)
        max_accYaw_index = list(accYaw_lst).index(max_accYaw)

        return (hwa_lst[min_accYaw_index], hwa_lst[max_accYaw_index])

    def accY_at_peak_N_calc(self, result_dict) -> Tuple[float, float]:
        accYaw_lst = np.array(result_dict["hwa"]["accYaw"] + result_dict["beta"]["accYaw"]).flatten()
        accY_lst = np.array(result_dict["hwa"]["accY"] + result_dict["beta"]["accY"]).flatten()

        min_accYaw = min(accYaw_lst)
        max_accYaw = max(accYaw_lst)

        min_accYaw_index = list(accYaw_lst).index(min_accYaw)
        max_accYaw_index = list(accYaw_lst).index(max_accYaw)

        return (accY_lst[min_accYaw_index], accY_lst[max_accYaw_index])

    def dN_dd_b0_calc(self, result_dict) -> float:
        tire_steer_lst = result_dict["beta"]["delta"]
        accYaw_lst = result_dict["beta"]["accYaw"]
        angle_lst = result_dict["beta"]["angle"]

        min_beta_mag = min(abs(np.array(angle_lst)))
        min_beta_index = list(abs(np.array(angle_lst))).index(min_beta_mag)

        tire_steer_line = [(x[0] + x[1]) / 2 for x in tire_steer_lst[min_beta_index]]
        accYaw_line = accYaw_lst[min_beta_index]

        beta_isoline = CubicSpline(tire_steer_line, accYaw_line)
        ddx_beta_isoline = beta_isoline.derivative()

        return ddx_beta_isoline(0)

    def dN_dd_b1_calc(self, result_dict) -> Tuple[float, float]:
        accY_lst = np.array(result_dict["beta"]["accY"]).flatten()
        beta_lst = np.array(result_dict["beta"]["beta"]).flatten()
        tire_steer_lst = [item for sublist in result_dict["beta"]["delta"] for item in sublist]

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)
        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)
        min_accY_tire_steer = (tire_steer_lst[min_accY_index][0] + tire_steer_lst[min_accY_index][1]) / 2
        max_accY_tire_steer = (tire_steer_lst[max_accY_index][0] + tire_steer_lst[max_accY_index][1]) / 2
        
        angle_lst = result_dict["beta"]["angle"]
        accYaw_lst = result_dict["beta"]["accYaw"]
        tire_steer_lst = result_dict["beta"]["delta"]

        min_accY_beta = beta_lst[min_accY_index]
        max_accY_beta = beta_lst[max_accY_index]
        min_accY_angle_index = list(angle_lst).index(min_accY_beta)
        max_accY_angle_index = list(angle_lst).index(max_accY_beta)

        min_accY_tire_steer_line = [(x[0] + x[1]) / 2 for x in tire_steer_lst[min_accY_angle_index]]
        min_accY_accYaw_line = accYaw_lst[min_accY_angle_index]

        max_accY_tire_steer_line = [(x[0] + x[1]) / 2 for x in tire_steer_lst[max_accY_angle_index]]
        max_accY_accYaw_line = accYaw_lst[max_accY_angle_index]

        min_accY_beta_isoline = CubicSpline(min_accY_tire_steer_line, min_accY_accYaw_line)
        max_accY_beta_isoline = CubicSpline(max_accY_tire_steer_line, max_accY_accYaw_line)
        min_accY_ddx_beta_isoline = min_accY_beta_isoline.derivative()
        max_accY_ddx_beta_isoline = max_accY_beta_isoline.derivative()

        return (min_accY_ddx_beta_isoline(min_accY_tire_steer), max_accY_ddx_beta_isoline(max_accY_tire_steer))
    
    def dN_db_d0_calc(self, result_dict) -> float:
        accYaw_lst = result_dict["hwa"]["accYaw"]
        beta_lst = result_dict["hwa"]["beta"]
        angle_lst = result_dict["hwa"]["angle"]

        min_hwa_mag = min(abs(np.array(angle_lst)))
        min_hwa_index = list(abs(np.array(angle_lst))).index(min_hwa_mag)

        beta_line = beta_lst[min_hwa_index]
        accYaw_line = accYaw_lst[min_hwa_index]

        hwa_isoline = CubicSpline(beta_line, accYaw_line)
        ddx_hwa_isoline = hwa_isoline.derivative()

        return ddx_hwa_isoline(0)

    def dN_db_d1_calc(self, result_dict) -> float:
        accY_lst = np.array(result_dict["hwa"]["accY"]).flatten()
        hwa_lst = np.array(result_dict["hwa"]["hwa"]).flatten()
        beta_lst = np.array(result_dict["hwa"]["beta"]).flatten()

        min_accY = min(accY_lst)
        max_accY = max(accY_lst)
        min_accY_index = list(accY_lst).index(min_accY)
        max_accY_index = list(accY_lst).index(max_accY)
        min_accY_beta = beta_lst[min_accY_index]
        max_accY_beta = beta_lst[max_accY_index]
        
        angle_lst = result_dict["hwa"]["angle"]
        accYaw_lst = result_dict["hwa"]["accYaw"]
        beta_lst = result_dict["hwa"]["beta"]

        min_accY_hwa = hwa_lst[min_accY_index]
        max_accY_hwa = hwa_lst[max_accY_index]
        min_accY_angle_index = list(angle_lst).index(min_accY_hwa)
        max_accY_angle_index = list(angle_lst).index(max_accY_hwa)

        min_accY_beta_line = beta_lst[min_accY_angle_index]
        min_accY_accYaw_line = accYaw_lst[min_accY_angle_index]

        max_accY_beta_line = beta_lst[max_accY_angle_index]
        max_accY_accYaw_line = accYaw_lst[max_accY_angle_index]

        min_accY_hwa_isoline = CubicSpline(min_accY_beta_line, min_accY_accYaw_line)
        max_accY_hwa_isoline = CubicSpline(max_accY_beta_line, max_accY_accYaw_line)
        min_accY_ddx_hwa_isoline = min_accY_hwa_isoline.derivative()
        max_accY_ddx_hwa_isoline = max_accY_hwa_isoline.derivative()

        return (min_accY_ddx_hwa_isoline(min_accY_beta), max_accY_ddx_hwa_isoline(max_accY_beta))

    def plot_cv_ymd(self, cv_states, cv_metrics):
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(1, 2, width_ratios=[0.75, 0.25])

        ### Graph ###
        ymd_ax = fig.add_subplot(gs[0])

        velX = cv_states["consts"]["velX"][0]
        ymd_ax.set_title(f"Constant Velocity: {velX}" + r" $m/s$ | Yaw Acceleration vs Lateral Acceleration")
        ymd_ax.set_xlabel(r"Lateral Acceleration $(G)$")
        ymd_ax.set_ylabel(r"Yaw Acceleration $(rad/s^{2})$")
        ymd_ax.axhline(c="gray", linewidth=0.5)
        ymd_ax.axvline(c="gray", linewidth=0.5)

        for i in range(len(cv_states["hwa"]["accY"])):
            ymd_ax.plot(cv_states["hwa"]["accY"][i], cv_states["hwa"]["accYaw"][i], c='b')
        for i in range(len(cv_states["beta"]["accY"])):
            ymd_ax.plot(cv_states["beta"]["accY"][i], cv_states["beta"]["accYaw"][i], c='r')

        custom_lines = [Line2D([0], [0], color='b', lw=1.5),
                        Line2D([0], [0], color='r', lw=1.5)]

        ymd_ax.legend(custom_lines, [r"Constant $\delta$ (hwa)", r"Constant $\beta$ (beta)"], loc='upper right')
        ymd_ax.grid()

        ### Tables ###
        table_ax = fig.add_subplot(gs[1])
        table_ax.axis("off")

        labels_table_data = []
        min_table_data = []
        max_table_data = []

        for key in self.min_table_dict.keys():
            min_table_data.append("{:.3f}".format(cv_metrics[key]))
        
        for key in self.max_table_dict.keys():
            labels_table_data.append(self.max_table_dict[key])
            max_table_data.append("{:.3f}".format(cv_metrics[key]))

        table_content = []

        table_content.append([None, None, "Left Half", "Right Half"])
        
        for i, label in enumerate(labels_table_data):
            if i < len(labels_table_data) - 2:
                table_content.append([label, self.unit_lst[i], min_table_data[i], max_table_data[i]])
            else:
                table_content.append([label, self.unit_lst[i], None, max_table_data[i]])

        metric_table = table_ax.table(
            cellText=table_content,
            loc="center",
            cellLoc="center",
            colWidths=[0.35, 0.20, 0.225, 0.225]
        )
        metric_table.scale(1.35, 3)
        metric_table.auto_set_font_size(True)

        for i in range(len(labels_table_data) + 1):
            for j in range(4):
                cell = metric_table[i, j]
                cell.visible_edges = "TB"
                cell.set_linewidth(1)
                cell.set_facecolor("#f0f0f0")
                cell.set_text_props(ha="center")
                
        for j in range(4):
            cell = metric_table[0, j]
            cell.visible_edges = "B"
            cell.set_text_props(ha="center") # weight="bold", 
        
        # for i in range(1, len(labels_table_data) + 1):
        #     cell = metric_table[i, 0]
        #     cell.set_text_props(ha="center")

        fig.tight_layout(pad=3.5)

        pos = ymd_ax.get_position()
        dy = -0.03375
        ymd_ax.set_position([pos.x0, pos.y0 + dy, pos.width, pos.height])

        return fig
    
    def plot_cr_ymd(self, cr_states, cr_metrics):
        fig = plt.figure(figsize=(14, 8.5), dpi=300)
        gs = gridspec.GridSpec(1, 2, width_ratios=[0.75, 0.25])

        ### Graph ###
        ymd_ax = fig.add_subplot(gs[0])

        radius = cr_states["consts"]["turn_radius"][0]
        ymd_ax.set_title(f"Constant Radius: {radius}" + r" $m$ | Yaw Acceleration vs Lateral Acceleration")
        ymd_ax.set_xlabel(r"Lateral Acceleration $(G)$")
        ymd_ax.set_ylabel(r"Yaw Acceleration $(rad/s^{2})$")
        ymd_ax.axhline(c="gray", linewidth=0.5)
        ymd_ax.axvline(c="gray", linewidth=0.5)

        for i in range(len(cr_states["hwa"]["accY"])):
            ymd_ax.plot(cr_states["hwa"]["accY"][i], cr_states["hwa"]["accYaw"][i], c='b')
        for i in range(len(cr_states["beta"]["accY"])):
            ymd_ax.plot(cr_states["beta"]["accY"][i], cr_states["beta"]["accYaw"][i], c='r')

        custom_lines = [Line2D([0], [0], color='b', lw=1.5),
                        Line2D([0], [0], color='r', lw=1.5)]

        ymd_ax.legend(custom_lines, [r"Constant $\delta$ (hwa)", r"Constant $\beta$ (beta)"], loc='upper right')
        ymd_ax.grid()

        ### Tables ###
        table_ax = fig.add_subplot(gs[1])
        table_ax.axis("off")

        labels_table_data = []
        min_table_data = []
        max_table_data = []

        for key in self.min_table_dict.keys():
            min_table_data.append("{:.3f}".format(cr_metrics[key]))
        
        for key in self.max_table_dict.keys():
            labels_table_data.append(self.max_table_dict[key])
            max_table_data.append("{:.3f}".format(cr_metrics[key]))

        table_content = []

        table_content.append([None, None, "Left Half", "Right Half"])
        
        for i, label in enumerate(labels_table_data):
            if i < len(labels_table_data) - 2:
                table_content.append([label, self.unit_lst[i], min_table_data[i], max_table_data[i]])
            else:
                table_content.append([label, self.unit_lst[i], None, max_table_data[i]])

        metric_table = table_ax.table(
            cellText=table_content,
            loc="center",
            cellLoc="center",
            colWidths=[0.35, 0.20, 0.225, 0.225]
        )
        metric_table.scale(1.35, 3)
        metric_table.auto_set_font_size(True)

        for i in range(len(labels_table_data) + 1):
            for j in range(4):
                cell = metric_table[i, j]
                cell.visible_edges = "TB"
                cell.set_linewidth(1)
                cell.set_facecolor("#f0f0f0")
                cell.set_text_props(ha="center")
                
        for j in range(4):
            cell = metric_table[0, j]
            cell.visible_edges = "B"
            cell.set_text_props(ha="center") # weight="bold", 
        
        # for i in range(1, len(labels_table_data) + 1):
        #     cell = metric_table[i, 0]
        #     cell.set_text_props(ha="center")

        fig.tight_layout(pad=3.5)

        pos = ymd_ax.get_position()
        dy = -0.03375
        ymd_ax.set_position([pos.x0, pos.y0 + dy, pos.width, pos.height])

        return fig