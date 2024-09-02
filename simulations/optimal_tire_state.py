from vehicle_model.vehicle_model import VehicleModel
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
import numpy as np


class OptimalTire:
    def __init__(self, vehicle: VehicleModel) -> None:
        self.tire = vehicle.FL_tire

        self.Fz_sweep = np.linspace(100, 1000, 50)
        self.alpha_sweep = np.linspace(0, 90 / 4, 50) * np.pi / 180
        self.kappa_sweep = np.linspace(0, 1 / 4, 50)
        self.gamma_sweep = np.linspace(-5, 5, 50) * np.pi / 180

    def optimal_state(self) -> None:
        Fz_lst = []
        gamma_lst = []
        mu_x_lst = []
        mu_y_lst = []

        max_mu_x_gamma_lst = []
        max_mu_y_gamma_lst = []
        max_mu_x_Fz_lst = []
        max_mu_y_Fz_lst = []

        for Fz in self.Fz_sweep:
            current_Fz_lst = []
            current_mu_x_lst = []
            current_mu_y_lst = []
            current_gamma_lst = []
            for gamma in self.gamma_sweep:
                mu_x, mu_y = self.get_mu(Fz=Fz, gamma=gamma)

                current_Fz_lst.append(Fz)
                current_mu_x_lst.append(mu_x)
                current_mu_y_lst.append(mu_y)
                current_gamma_lst.append(gamma)

                Fz_lst.append(Fz)
                gamma_lst.append(gamma)
                mu_x_lst.append(mu_x)
                mu_y_lst.append(mu_y)
            
            max_mu_x = max(current_mu_x_lst)
            max_mu_y = max(current_mu_y_lst)
            max_mu_x_index = current_mu_x_lst.index(max_mu_x)
            max_mu_y_index = current_mu_y_lst.index(max_mu_y)
            max_mu_x_gamma = current_gamma_lst[max_mu_x_index]
            max_mu_y_gamma = current_gamma_lst[max_mu_y_index]
            
            max_mu_x_Fz = current_Fz_lst[max_mu_x_index]
            max_mu_y_Fz = current_Fz_lst[max_mu_y_index]

            max_mu_x_gamma_lst.append(max_mu_x_gamma)
            max_mu_y_gamma_lst.append(max_mu_y_gamma)
            max_mu_x_Fz_lst.append(max_mu_x_Fz)
            max_mu_y_Fz_lst.append(max_mu_y_Fz)


        for i in range(len(max_mu_x_Fz_lst)):
            plt.scatter(max_mu_x_Fz_lst[i], max_mu_x_gamma_lst[i])
            # plt.plot(max_mu_y_Fz_lst[i], max_mu_y_gamma_lst[i])
        plt.show()

        fig = plt.figure(figsize=[11, 8.5])
        fig.suptitle("Mu(Fz, gamma)", fontsize=20)

        Fz = np.array(Fz_lst).reshape(len(self.Fz_sweep), len(self.gamma_sweep))
        gamma = np.array(gamma_lst).reshape(len(self.Fz_sweep), len(self.gamma_sweep))
        mu_x = np.array(mu_x_lst).reshape(len(self.Fz_sweep), len(self.gamma_sweep))
        mu_y = np.array(mu_y_lst).reshape(len(self.Fz_sweep), len(self.gamma_sweep))

        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot_surface(Fz, gamma, mu_x)

        self.format_fig(ax=ax,
                        title=f"Mu_x(Fz, gamma)",
                        xlabel="Fz (N)",
                        ylabel="Gamma (deg)",
                        zlabel="Mu_x (-)",
                        title_size=8,
                        label_size=6)

        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.plot_surface(Fz, gamma, mu_y)

        self.format_fig(ax=ax,
                        title=f"Mu_y(Fz, gamma)",
                        xlabel="Fz (N)",
                        ylabel="Gamma (deg)",
                        zlabel="Mu_y (-)",
                        title_size=8,
                        label_size=6)
        
        plt.show()

    def get_mu(self, Fz: float, gamma: float):
        Fx = self.tire.tire_eval(FZ=Fz, alpha=0, kappa=self.kappa_sweep, gamma=gamma)[0]
        Fy = self.tire.tire_eval(FZ=Fz, alpha=self.alpha_sweep, kappa=0, gamma=gamma)[1]

        mu_x = max(Fx) / Fz
        mu_y = max(Fy) / Fz
        # mu_x, mu_y = self.tire.get_mu(FZ=Fz, gamma=gamma)

        return [mu_x, mu_y]

    def format_fig(self, ax: Axes, title: str, xlabel: str, ylabel: str, zlabel: str, title_size: int, label_size: int) -> Figure:

        ax.set_title(title, fontsize=title_size)
        ax.set_xlabel(xlabel, fontsize=label_size)
        ax.set_ylabel(ylabel, fontsize=label_size)
        ax.set_zlabel(zlabel, fontsize=label_size)
        ax.tick_params(axis='both', labelsize=label_size)