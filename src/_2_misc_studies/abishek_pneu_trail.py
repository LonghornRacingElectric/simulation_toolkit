from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import numpy as np

# Load tire model
tire = MF52(
    tire_name="",
    file_path="./src/_1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir"
)

# Sweeps
Fz_sweep = np.linspace(100, 1600, 50)   # vertical load [N]
alpha_sweep = np.linspace(-30, 30, 50) * np.pi / 180  # slip angle [deg]

# Create meshgrid
Alpha, Fz = np.meshgrid(alpha_sweep, Fz_sweep)

# Compute pneumatic trail
pneu_trail = np.zeros_like(Alpha)
for i in range(Fz.shape[0]):
    for j in range(Fz.shape[1]):
        pneu_trail[i, j] = tire.get_pneu_trail(FZ=Fz[i, j], alpha=Alpha[i, j])

# Plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")
surf = ax.plot_surface(Alpha * 180 / np.pi, Fz, pneu_trail, cmap="viridis")

ax.set_xlabel("Slip Angle α [deg]")
ax.set_ylabel("Vertical Load Fz [N]")
ax.set_zlabel("Pneumatic Trail [m]")
ax.set_title("Pneumatic Trail vs Slip Angle & Vertical Load")

fig.colorbar(surf, shrink=0.5, aspect=10, label="Pneumatic Trail [m]")
plt.show()
