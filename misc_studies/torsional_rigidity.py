import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Assumptions: Instant centers at the ground

def eval(KsF: float = 450, KsR: float = 450, kC: float = 1e9):
    KsF = KsF * 175.127 # N/m
    MRF = 1.2
    KsR = KsR * 175.127 # N/m
    MRR = 1.2
    kC = kC # Nm/rad
    cg_bias_F = 0.50

    l = 61 * 0.0254
    kF = 1 / 2 * (50 * 0.0254)**2 * KsF / MRF**2
    kR = 1 / 2 * (50 * 0.0254)**2 * KsR / MRR**2
    a = l * (1 - cg_bias_F)
    b = l * cg_bias_F

    dFzF_simp = (kF / (kF + (kR * kC) / (kR + kC)) * b / l + (kF * kC) / (kF + kC) / (kR + (kF * kC) / (kF + kC)) * a / l)

    dFzR_simp = ((kR * kC) / (kR + kC) / (kF + (kR * kC) / (kR + kC)) * b / l + (kR / (kR + (kF * kC) / (kF + kC))) * a / l)

    dFz_tot = dFzF_simp + dFzR_simp

    nom_LLTD = KsF / (KsF + KsR)
    compl_LLTD = dFzF_simp / dFz_tot
    
    return (compl_LLTD - nom_LLTD) / nom_LLTD * 100

def eval_nom(KsF: float = 450, KsR: float = 450, kC: float = 1e9):
    return KsF / (KsF + KsR)

KsR_sweep = np.linspace(250, 650, 100)
kC_sweep = np.linspace(1, 500000, 100)

KsR_sweep_mesh, kC_sweep_mesh = np.meshgrid(KsR_sweep, kC_sweep)
perc_diff_mesh = eval(KsR=KsR_sweep_mesh, kC=kC_sweep_mesh)

nom_LLTD_sweep = [eval_nom(x) for x in KsR_sweep]

nom_LLTD_mesh, y = np.meshgrid(nom_LLTD_sweep, kC_sweep)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(nom_LLTD_mesh, kC_sweep_mesh, perc_diff_mesh, cmap='viridis')

# Add labels and title
ax.set_xlabel('Rigid LLTD')
ax.set_ylabel('Chassis Torsional Stiffness (Nm/rad)')
ax.set_zlabel('LLTD Percent Difference')
plt.title('LLTD Percent Difference w/ vs w/o Chassis Compliance')

plt.show()