from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import numpy as np

# Load tire model
tire = MF52(
    tire_name="",
    file_path="./src/_1_model_inputs/Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir"
)

FZNOMIN = tire.get_FNOMIN()

# Sweeps
alpha_sweep = np.linspace(-20, 20, 100) * np.pi / 180  # Slip angle (rad)
gamma_sweep = np.linspace(-4, 0, 5) * np.pi / 180     # Camber angle (rad)

Fy_lst_lst = []

for gamma in gamma_sweep:
    Fy_lst = []
    for alpha in alpha_sweep:
        Fy = tire.tire_eval(FZ=FZNOMIN, alpha=alpha, kappa=0, gamma=gamma)[1]
        Fy_lst.append(Fy)
    
    Fy_lst_lst.append(Fy_lst)


# Create 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.gca()

for Fy_vals, gamma_val in zip(Fy_lst_lst, gamma_sweep):
    ax.plot(alpha_sweep * 180 / np.pi, Fy_vals, label=rf"$\gamma = {gamma_val * 180 / np.pi}$")

ax.grid()
ax.set_xlim([0, max(alpha_sweep) * 180 / np.pi])
ax.set_ylim([0, max(Fy_lst_lst[0]) * 1.125])
ax.legend()

ax.set_title(r"$ F_{y} \: vs \: \alpha$" + f" at Fz = {FZNOMIN}")
ax.set_xlabel(r"Slip Angle, $\alpha$ ($deg$)")
ax.set_ylabel(r"Lateral Force, $F_{y}$ ($N$)")
plt.show()