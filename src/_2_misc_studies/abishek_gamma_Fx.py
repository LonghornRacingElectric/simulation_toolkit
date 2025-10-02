from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import numpy as np

# Load tire model
tire = MF52(
    tire_name="",
    file_path="./src/_1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir"
)

FZNOMIN = tire.get_FNOMIN()

# Sweeps
kappa_sweep = np.linspace(-0.10, 0.10, 100)
gamma_sweep = np.linspace(-4, 0, 5) * np.pi / 180

Fx_lst_lst = []

for gamma in gamma_sweep:
    Fx_lst = []
    for kappa in kappa_sweep:
        Fx = tire.tire_eval(FZ=FZNOMIN, alpha=1e-6, kappa=kappa, gamma=gamma)[0]
        Fx_lst.append(Fx)
    
    Fx_lst_lst.append(Fx_lst)

# Create 3D plot
fig = plt.figure(figsize=(10, 7))
ax = fig.gca()

for Fx_vals, gamma_val in zip(Fx_lst_lst, gamma_sweep):
    print(Fx_vals)
    ax.plot(kappa_sweep, Fx_vals, label=rf"$\gamma = {gamma_val * 180 / np.pi}$")

ax.grid()
ax.set_xlim([0, max(kappa_sweep)])
ax.set_ylim([0, max(Fx_lst_lst[0]) * 1.125])
ax.legend()

ax.set_title(r"$ F_{x} \: vs \: \kappa$" + f" at Fz = {FZNOMIN}")
ax.set_xlabel(r"Slip Ratio, $\kappa$")
ax.set_ylabel(r"Longitudinal Force, $F_{x}$ ($N$)")
plt.show()