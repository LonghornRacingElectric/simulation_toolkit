from matplotlib.backends.backend_pdf import PdfPages
from LHR_tire_toolkit.MF52 import MF52
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
from typing import Sequence
import numpy as np


class TirePlotting:
    def __init__(self, tire: MF52) -> None:
        self.tire = tire
    
    def get_mesh_fig(self, Fz_nomin: float, gamma: float, Fz_sweep: Sequence[float] | float, kappa_sweep: Sequence[float] | None = None, 
                      alpha_sweep: Sequence[float] | None = None) -> Sequence[Figure]:
        
        # Create figure
        fig = plt.figure(figsize=[11, 8.5])
        fig.suptitle("Tire Fit Plots", fontsize=20)
        
        ### Create subplots ###

        # Pure slip Fx
        ax = fig.add_subplot(2, 3, 1, projection='3d')
        Fz, kappa = np.meshgrid(Fz_sweep, kappa_sweep)
        Fx = np.array(self.tire.tire_eval(FZ=Fz, alpha=0, kappa=kappa, gamma=gamma)[0])
        
        ax.plot_surface(Fz, kappa, Fx)

        self.format_fig(ax=ax,
                        title=f"Fx(Fz, alpha=0, kappa, gamma={round(gamma, 3)})",
                        xlabel="Fz (N)",
                        ylabel="Kappa (-)",
                        zlabel="Fx (N)",
                        title_size=8,
                        label_size=6)

        # Pure slip Fy
        ax = fig.add_subplot(2, 3, 2, projection='3d')
        Fz, alpha = np.meshgrid(Fz_sweep, alpha_sweep)
        Fy = np.array(self.tire.tire_eval(FZ=Fz, alpha=alpha, kappa=0, gamma=gamma)[1])
        Mz = np.array(self.tire.tire_eval(FZ=Fz, alpha=alpha, kappa=0, gamma=gamma)[5])

        ax.plot_surface(Fz, alpha, Fy)

        self.format_fig(ax=ax,
                        title=f"Fy(Fz, alpha, kappa=0, gamma={round(gamma, 3)})",
                        xlabel="Fz (N)",
                        ylabel="Alpha (rad)",
                        zlabel="Fy (N)",
                        title_size=8,
                        label_size=6)
        
        # Pure slip Mz
        ax = fig.add_subplot(2, 3, 3, projection='3d')
        ax.plot_surface(Fz, alpha, Mz)

        self.format_fig(ax=ax,
                        title=f"Mz(Fz, alpha, kappa=0, gamma={round(gamma, 3)})",
                        xlabel="Fz (N)",
                        ylabel="Alpha (rad)",
                        zlabel="Mz (Nm)",
                        title_size=8,
                        label_size=6)
        
        # Combined slip initialization
        alpha, kappa = np.meshgrid(alpha_sweep, kappa_sweep)

        Fx = self.tire.tire_eval(FZ=Fz, alpha=alpha, kappa=kappa, gamma=gamma)[0]
        Fy = self.tire.tire_eval(FZ=Fz, alpha=alpha, kappa=kappa, gamma=gamma)[1]
        Mz = self.tire.tire_eval(FZ=Fz, alpha=alpha, kappa=kappa, gamma=gamma)[5]

        # Combined slip Fx
        ax = fig.add_subplot(2, 3, 4, projection='3d')
        ax.plot_surface(alpha, kappa, Fx)

        self.format_fig(ax=ax,
                        title=f"Fx(Fz={Fz_nomin}, alpha, kappa, gamma={round(gamma, 3)})",
                        xlabel="Alpha (rad)",
                        ylabel="Kappa (-)",
                        zlabel="Fx (N)",
                        title_size=8,
                        label_size=6)

        # Combined slip Fy
        ax = fig.add_subplot(2, 3, 5, projection='3d')
        ax.plot_surface(alpha, kappa, Fy)

        self.format_fig(ax=ax,
                        title=f"Fy(Fz={Fz_nomin}, alpha, kappa, gamma={round(gamma, 3)})",
                        xlabel="Alpha (rad)",
                        ylabel="Kappa (-)",
                        zlabel="Fy (N)",
                        title_size=8,
                        label_size=6)
        
        # Combined slip Mz
        ax = fig.add_subplot(2, 3, 6, projection='3d')
        ax.plot_surface(alpha, kappa, Fx)

        self.format_fig(ax=ax,
                        title=f"Mz(Fz={Fz_nomin}, alpha, kappa, gamma={round(gamma, 3)})",
                        xlabel="Alpha (rad)",
                        ylabel="Kappa (-)",
                        zlabel="Mz (Nm)",
                        title_size=8,
                        label_size=6)
        
        return fig
            
    def get_friction_ellipse_fig(self, Fz_nomin: float | int, gamma: float, kappa_sweep: Sequence[float], alpha_sweep: Sequence[float]) -> Sequence[Figure]:
        # Plot setup
        fig = plt.figure(figsize=[11, 8.5])
        ax = fig.gca()
        ax.set_aspect('equal')
        ax.set_xticks(np.arange(-4, 4, 0.5), minor=True)
        ax.set_yticks(np.arange(-4, 4, 0.5), minor=True)
        ax.grid(which='minor', alpha=0.5)
        ax.grid(which='major', alpha=0.5)

        # Set up sweep tracking
        sweeps = {"kappa_total": [],
                  "kappa_single": [],
                  "kappa_mu_y_total": [],
                  "kappa_mu_y_single": [],
                  "alpha_total": [],
                  "alpha_single": [],
                  "alpha_mu_x_total": [],
                  "alpha_mu_x_single": []}
        
        # Combined Fx sweep
        for kappa in kappa_sweep:
            sweeps["kappa_single"] = []
            sweeps["kappa_mu_y_single"] = []
            for alpha in alpha_sweep:
                FX_comb = self.tire.tire_eval(FZ=Fz_nomin, alpha=alpha, kappa=kappa, gamma=gamma)[0]
                FY_comb = self.tire.tire_eval(FZ=Fz_nomin, alpha=alpha, kappa=kappa, gamma=gamma)[1]
                sweeps["kappa_single"].append(FX_comb/Fz_nomin)
                sweeps["kappa_mu_y_single"].append(FY_comb/Fz_nomin)

            sweeps["kappa_total"].append(sweeps["kappa_single"])
            sweeps["kappa_mu_y_total"].append(sweeps["kappa_mu_y_single"])

        # Combined Fy sweep
        for alpha in alpha_sweep:
            sweeps["alpha_single"] = []
            sweeps["alpha_mu_x_single"] = []
            for kappa in kappa_sweep:
                FX_comb = self.tire.tire_eval(FZ=Fz_nomin, alpha=alpha, kappa=kappa, gamma=gamma)[0]
                FY_comb = self.tire.tire_eval(FZ=Fz_nomin, alpha=alpha, kappa=kappa, gamma=gamma)[1]
                sweeps["alpha_single"].append(FY_comb/Fz_nomin)
                sweeps["alpha_mu_x_single"].append(FX_comb/Fz_nomin)

            sweeps["alpha_total"].append(sweeps["alpha_single"])
            sweeps["alpha_mu_x_total"].append(sweeps["alpha_mu_x_single"])
        
        # Plot isolines
        for i, isoline in enumerate(sweeps["kappa_total"]):
            plt.plot(sweeps["kappa_mu_y_total"][i], isoline, c='b')

        for i, isoline in enumerate(sweeps["alpha_total"]):
            plt.plot(isoline, sweeps["alpha_mu_x_total"][i], c='r')

        ax.set_title(f"Friction Ellipse(Fz={Fz_nomin} N)")
        ax.set_xlabel("Fy/Fz")
        ax.set_ylabel("Fx/Fz")
        leg = ax.legend(["SA Isolines", "SR Isolines"], loc='upper right')
        leg.legend_handles[0].set_color('red')
        leg.legend_handles[1].set_color('blue')
    
        return fig

    def save_pdf(self, figs: Sequence[Figure], save_path: str) -> None:
        p = PdfPages(save_path)

        for page in figs:
            page.savefig(p, format="pdf")

        p.close()

    def format_fig(self, ax: Axes, title: str, xlabel: str, ylabel: str, zlabel: str, title_size: int, label_size: int) -> Figure:

        ax.set_title(title, fontsize=title_size)
        ax.set_xlabel(xlabel, fontsize=label_size)
        ax.set_ylabel(ylabel, fontsize=label_size)
        ax.set_zlabel(zlabel, fontsize=label_size)
        ax.tick_params(axis='both', labelsize=label_size)

### This is dumb, but the script is located in the same file as the class
### I'll eventually move this to the tire analysis package

if __name__ == "__main__":
    tire = MF52(tire_name='test', file_path='model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir')
    tire_plotter = TirePlotting(tire=tire)

    # Fit plot parameters
    FZ_sweep = np.linspace(10, 1000, 50)
    kappa_sweep_mesh = np.linspace(-0.25, 0.25, 50)
    alpha_sweep_mesh = np.linspace(-25 * np.pi / 180, 25 * np.pi / 180, 50)

    # Friction ellipse parameters
    kappa_sweep = np.linspace(-0.10, 0.10, 30)
    alpha_sweep = np.linspace(-10 * np.pi / 180, 10 * np.pi / 180, 30)

    mesh_plots = tire_plotter.get_mesh_fig(Fz_nomin=654, gamma=0 * np.pi / 180, Fz_sweep=FZ_sweep, kappa_sweep=kappa_sweep_mesh, alpha_sweep=alpha_sweep_mesh)
    ellipse_plot = tire_plotter.get_friction_ellipse_fig(Fz_nomin=654, gamma=0, kappa_sweep=kappa_sweep, alpha_sweep=alpha_sweep)

    tire_plotter.save_pdf(figs=[mesh_plots, ellipse_plot], save_path="./outputs/Tire_Fit_Plots.pdf")