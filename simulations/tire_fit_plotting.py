from matplotlib.backends.backend_pdf import PdfPages
from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import numpy as np

test = MF52(tire_name='test', file_path='model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir')

fig = plt.figure(figsize=[11, 8.5])

fig.suptitle(t="General Tire Fits", fontsize='20')

FZ_sweep = np.linspace(10, 1000, 50)
kappa_sweep = np.linspace(-1/8, 1/8, 50)
FZ, kappa = np.meshgrid(FZ_sweep, kappa_sweep)
FX = np.array(test.tire_eval(FZ=FZ, alpha=0, kappa=kappa, gamma=0 * np.pi / 180)[0])

# Fx
ax_1 = fig.add_subplot(2, 3, 1, projection='3d')
fig.add_axes(ax_1)

ax_1.plot_surface(FZ, kappa, FX)

ax_1.set_xlabel('Fz (N)', fontsize='8')
ax_1.set_ylabel('Kappa (-)', fontsize='8')
ax_1.set_zlabel('Fx (N)', fontsize='8')
ax_1.set_title('Fx(Fz, alpha=0, kappa, gamma=0)', fontsize='10')

ax_1.tick_params(axis='both', labelsize=8)

##########

FZ_sweep = np.linspace(10, 1000, 50)
alpha_sweep = np.linspace(-np.pi/8, np.pi/8, 50)
FZ, alpha = np.meshgrid(FZ_sweep, alpha_sweep)
FY_1 = np.array(test.tire_eval(FZ=FZ, alpha=alpha, kappa=0, gamma=0 * np.pi / 180)[1])
FY_2 = np.array(test.tire_eval(FZ=FZ, alpha=alpha, kappa=0, gamma=-5 * np.pi / 180)[1])

# Fy
ax_2 = fig.add_subplot(2, 3, 2, projection='3d')
fig.add_axes(ax_1)

ax_2.plot_surface(FZ, alpha, FY_1)
ax_2.plot_surface(FZ, alpha, FY_2)
ax_2.legend(["gamma=0 deg", "gamma=-5 deg"])

ax_2.set_xlabel('Fz (N)', fontsize='8')
ax_2.set_ylabel('Alpha (rad)', fontsize='8')
ax_2.set_zlabel('Fy (N)', fontsize='8')
ax_2.set_title('Fy(Fz, alpha, kappa=0, gamma=[0,-5])', fontsize='10')

ax_2.tick_params(axis='both', labelsize=8)

##########

FZ_sweep = np.linspace(10, 1000, 50)
alpha_sweep = np.linspace(-np.pi/8, np.pi/8, 50)
FZ, alpha = np.meshgrid(FZ_sweep, alpha_sweep)
MZ_1 = np.array(test.tire_eval(FZ=FZ, alpha=alpha, kappa=0, gamma=0 * np.pi / 180)[5])
MZ_2 = np.array(test.tire_eval(FZ=FZ, alpha=alpha, kappa=0, gamma=-5 * np.pi / 180)[5])

# Mz
ax_3 = fig.add_subplot(2, 3, 3, projection='3d')
fig.add_axes(ax_3)

ax_3.plot_surface(FZ, alpha, MZ_1)
ax_3.plot_surface(FZ, alpha, MZ_2)

ax_3.set_xlabel('Fz (N)', fontsize='8')
ax_3.set_ylabel('Alpha (rad)', fontsize='8')
ax_3.set_zlabel('Mz (Nm)', fontsize='8')
ax_3.set_title('Mz(Fz, alpha, kappa=0, gamma=[0, -5])', fontsize='10')
ax_3.legend(["gamma=0 deg", "gamma=-5 deg"])

ax_3.tick_params(axis='both', labelsize=8)

#########

model_SA_data = np.linspace(-20 * np.pi / 180, 20 * np.pi / 180, 1000)
model_SR_data = np.linspace(-0.5, 0.5, 1000)

SA, SR = np.meshgrid(model_SA_data, model_SR_data)

FX_comb = test.tire_eval(FZ=1000, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[0]
FY_comb = test.tire_eval(FZ=1000, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[1]
MZ_comb = test.tire_eval(FZ=1000, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[5]

# Combined Fx
ax_4 = fig.add_subplot(2, 3, 4, projection='3d')
fig.add_axes(ax_4)

ax_4.plot_surface(SA, SR, FX_comb)
ax_4.set_xlabel('Alpha (rad)', fontsize='8')
ax_4.set_ylabel('Kappa (-)', fontsize='8')
ax_4.set_zlabel('Fx (N)', fontsize='8')
ax_4.set_title('Fx(Fz=1000, alpha, kappa, gamma=0)', fontsize='10')

ax_4.tick_params(axis='both', labelsize=8)

# Combined Fy
ax_5 = fig.add_subplot(2, 3, 5, projection='3d')
fig.add_axes(ax_5)

ax_5.plot_surface(SA, SR, FY_comb)
ax_5.set_xlabel('Alpha (rad)', fontsize='8')
ax_5.set_ylabel('Kappa (-)', fontsize='8')
ax_5.set_zlabel('Fy (N)', fontsize='8')
ax_5.set_title('Fy(Fz=1000, alpha, kappa, gamma=0)', fontsize='10')

ax_5.tick_params(axis='both', labelsize=8)

##########

# Combined Mz
ax_6 = fig.add_subplot(2, 3, 6, projection='3d')
fig.add_axes(ax_6)

ax_6.plot_surface(SA, SR, MZ_comb)
ax_6.set_xlabel('Alpha (rad)', fontsize='8')
ax_6.set_ylabel('Kappa (-)', fontsize='8')
ax_6.set_zlabel('Mz (Nm)', fontsize='8')
ax_6.set_title('Mz(Fz=1000, alpha, kappa, gamma=0)', fontsize='10')

ax_6.tick_params(axis='both', labelsize=8)

##########

# Friction Ellipse

fig_2 = plt.figure(figsize=[11, 8.5])

FZ = 654
model_SA_data = np.linspace(-10 * np.pi / 180, 10 * np.pi / 180, 30)
model_SR_data = np.linspace(-0.10, 0.10, 30)

SA_isoline_sweep = []
SA_isoline = []
SR_mu_vals_sweep = []
SR_mu_vals = []

for SA in model_SA_data:
    SA_isoline = []
    SR_mu_vals = []
    for SR in model_SR_data:
        FX_comb = test.tire_eval(FZ=FZ, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[0]
        FY_comb = test.tire_eval(FZ=FZ, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[1]
        SR_mu_vals.append(FX_comb/FZ)
        SA_isoline.append(FY_comb/FZ)

    SR_mu_vals_sweep.append(SR_mu_vals)
    SA_isoline_sweep.append(SA_isoline)

SR_isoline_sweep = []
SR_isoline = []
SA_mu_vals_sweep = []
SA_mu_vals = []

for SR in model_SR_data:
    SR_isoline = []
    SA_mu_vals = []
    for SA in model_SA_data:
        FX_comb = test.tire_eval(FZ=FZ, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[0]
        FY_comb = test.tire_eval(FZ=FZ, alpha=SA, kappa=SR, gamma=0 * np.pi / 180)[1]
        SA_mu_vals.append(FY_comb/FZ)
        SR_isoline.append(FX_comb/FZ)

    SA_mu_vals_sweep.append(SA_mu_vals)
    SR_isoline_sweep.append(SR_isoline)

for i, isoline in enumerate(SA_isoline_sweep):
    plt.plot(SR_mu_vals_sweep[i], isoline, c='r')

for i, isoline in enumerate(SR_isoline_sweep):
    plt.plot(isoline, SA_mu_vals_sweep[i], c='b')

plt.title("Friction Ellipse")
plt.xlabel("Fx/Fz")
plt.ylabel("Fy/Fz")
leg = plt.legend(["SA Isolines", "SR Isolines"], loc='upper right')
leg.legend_handles[0].set_color('red')
leg.legend_handles[1].set_color('blue')

# Save as pdf
p = PdfPages(f"./outputs/General_Tire_Plots.pdf")

for fig in [fig, fig_2]:
    fig.savefig(p, format = "pdf")

p.close()