from matplotlib.figure import Figure
from matplotlib.axes import Axes
import matplotlib.pyplot as plt
from typing import Sequence
import pandas as pd
import numpy as np

# This is a glorified calculator. No actual code here.

spring_rates = np.array([150, 175, 200, 225, 250, 300, 350, 400, 450, 500, 550, 600, 650]) * 175.12683464567 # N/m
tire_K = 98947 # N/m
sprung_mass = 238 - 16.328 - 18.611 # kg
ride_height = 2 * 0.0254 * 1000 # mm

cg_bias_Fr = 0.50
cg_bias_Lt = 0.50

FL_mass = sprung_mass * cg_bias_Fr * cg_bias_Lt
FR_mass = sprung_mass * cg_bias_Fr * (1 - cg_bias_Lt)
RL_mass = sprung_mass * (1 - cg_bias_Fr) * cg_bias_Lt
RR_mass = sprung_mass * (1 - cg_bias_Fr) * (1 - cg_bias_Lt)

# Min MR
Fr_MR_min = 1.185789499
Rr_MR_min = 1.623060443

Fr_Kw_min = spring_rates / Fr_MR_min**2
Rr_Kw_min = spring_rates / Rr_MR_min**2

Fr_Kride_min = (1/Fr_Kw_min + 1/tire_K)**(-1)
Rr_Kride_min = (1/Rr_Kw_min + 1/tire_K)**(-1)

Fr_Freq_min = 1 / (2 * np.pi) * np.sqrt(Fr_Kride_min / FL_mass)
Rr_Freq_min = 1 / (2 * np.pi) * np.sqrt(Rr_Kride_min / RL_mass)

# Nominal MR
Fr_MR_nom = 1.273846638
Rr_MR_nom = 1.776764199

Fr_Kw_nom = spring_rates / Fr_MR_nom**2
Rr_Kw_nom = spring_rates / Rr_MR_nom**2

Fr_Kride_nom = (1/Fr_Kw_nom + 1/tire_K)**(-1)
Rr_Kride_nom = (1/Rr_Kw_nom + 1/tire_K)**(-1)

Fr_Freq_nom = 1 / (2 * np.pi) * np.sqrt(Fr_Kride_nom / FL_mass)
Rr_Freq_nom = 1 / (2 * np.pi) * np.sqrt(Rr_Kride_nom / RL_mass)

print(spring_rates / 175.12683464567)
print(Fr_Freq_nom)
print(Rr_Freq_nom)

# Max MR
Fr_MR_max = 1.313922321
Rr_MR_max = 1.849249205

Fr_Kw_max = spring_rates / Fr_MR_max**2
Rr_Kw_max = spring_rates / Rr_MR_max**2

Fr_Kride_max = (1/Fr_Kw_max + 1/tire_K)**(-1)
Rr_Kride_max = (1/Rr_Kw_max + 1/tire_K)**(-1)

Fr_Freq_max = 1 / (2 * np.pi) * np.sqrt(Fr_Kride_max / FL_mass)
Rr_Freq_max = 1 / (2 * np.pi) * np.sqrt(Rr_Kride_max / RL_mass)

# state_output = pd.read_csv("./model_inputs/tire_states_35_mps.csv")
state_output = pd.read_csv("./model_inputs/tire_states_15_mps.csv")
max_Fr_corner_Fz = max(state_output["FL_Fz"].tolist() + state_output["FR_Fz"].tolist())
max_Rr_corner_Fz = max(state_output["RL_Fz"].tolist() + state_output["RR_Fz"].tolist())

# Min
max_Fr_corner_z_min = max_Fr_corner_Fz / Fr_Kride_min * 1000
max_Rr_corner_z_min = max_Rr_corner_Fz / Rr_Kride_min * 1000

# Nominal
max_Fr_corner_z_nom = max_Fr_corner_Fz / Fr_Kride_nom * 1000
max_Rr_corner_z_nom = max_Rr_corner_Fz / Rr_Kride_nom * 1000

# Max
max_Fr_corner_z_max = max_Fr_corner_Fz / Fr_Kride_max * 1000
max_Rr_corner_z_max = max_Rr_corner_Fz / Rr_Kride_max * 1000

# Plotting
fig, axs = plt.subplots(nrows=3, ncols=2)

### Min MR ###

# Ride freq setup
axs[0, 0].set_title("Ride Rate vs Coil Spring Rate")
axs[0, 0].set_xlabel("Coil Spring Rate (lbf/in)")
axs[0, 0].set_ylabel("Ride Rate (Hz)")
axs[0, 0].grid()

# Max displacement setup
axs[0, 1].set_title("Max Corner Displacement vs Coil Spring Rate")
axs[0, 1].set_xlabel("Coil Spring Rate (lbf/in)")
axs[0, 1].set_ylabel("Max Corner Displacement (mm)")
axs[0, 1].grid()

# Ride freq plotting
axs[0, 0].plot(spring_rates / 175.12683464567, Fr_Freq_min, c="k", label="Front Corner")
axs[0, 0].plot(spring_rates / 175.12683464567, Rr_Freq_min, c="m", label="Rear Corner")
axs[0, 0].scatter(spring_rates / 175.12683464567, Fr_Freq_min, c="k")
axs[0, 0].scatter(spring_rates / 175.12683464567, Rr_Freq_min, c="m")
axs[0, 0].legend()

# Max displacement plotting
axs[0, 1].plot(spring_rates / 175.12683464567, max_Fr_corner_z_min, c="k", label="Front Corner")
axs[0, 1].plot(spring_rates / 175.12683464567, max_Rr_corner_z_min, c="m", label="Rear Corner")
axs[0, 1].plot(spring_rates / 175.12683464567, [ride_height for _ in range(len(spring_rates))], c="r")
axs[0, 1].scatter(spring_rates / 175.12683464567, max_Fr_corner_z_min, c="k")
axs[0, 1].scatter(spring_rates / 175.12683464567, max_Rr_corner_z_min, c="m")
axs[0, 1].legend()

### Nominal MR ###

# Ride freq setup
axs[1, 0].set_title("Ride Rate vs Coil Spring Rate")
axs[1, 0].set_xlabel("Coil Spring Rate (lbf/in)")
axs[1, 0].set_ylabel("Ride Rate (Hz)")
axs[1, 0].grid()

# Max displacement setup
axs[1, 1].set_title("Max Corner Displacement vs Coil Spring Rate")
axs[1, 1].set_xlabel("Coil Spring Rate (lbf/in)")
axs[1, 1].set_ylabel("Max Corner Displacement (mm)")
axs[1, 1].grid()

# Ride freq plotting
axs[1, 0].plot(spring_rates / 175.12683464567, Fr_Freq_nom, c="k", label="Front Corner")
axs[1, 0].plot(spring_rates / 175.12683464567, Rr_Freq_nom, c="m", label="Rear Corner")
axs[1, 0].scatter(spring_rates / 175.12683464567, Fr_Freq_nom, c="k")
axs[1, 0].scatter(spring_rates / 175.12683464567, Rr_Freq_nom, c="m")
axs[1, 0].legend()

# Max displacement plotting
axs[1, 1].plot(spring_rates / 175.12683464567, max_Fr_corner_z_nom, c="k", label="Front Corner")
axs[1, 1].plot(spring_rates / 175.12683464567, max_Rr_corner_z_nom, c="m", label="Rear Corner")
axs[1, 1].plot(spring_rates / 175.12683464567, [ride_height for _ in range(len(spring_rates))], c="r")
axs[1, 1].scatter(spring_rates / 175.12683464567, max_Fr_corner_z_nom, c="k")
axs[1, 1].scatter(spring_rates / 175.12683464567, max_Rr_corner_z_nom, c="m")
axs[1, 1].legend()

### Max MR ###

# Ride freq setup
axs[2, 0].set_title("Ride Rate vs Coil Spring Rate")
axs[2, 0].set_xlabel("Coil Spring Rate (lbf/in)")
axs[2, 0].set_ylabel("Ride Rate (Hz)")
axs[2, 0].grid()

# Max displacement setup
axs[2, 1].set_title("Max Corner Displacement vs Coil Spring Rate")
axs[2, 1].set_xlabel("Coil Spring Rate (lbf/in)")
axs[2, 1].set_ylabel("Max Corner Displacement (mm)")
axs[2, 1].grid()

# Ride freq plotting
axs[2, 0].plot(spring_rates / 175.12683464567, Fr_Freq_max, c="k", label="Front Corner")
axs[2, 0].plot(spring_rates / 175.12683464567, Rr_Freq_max, c="m", label="Rear Corner")
axs[2, 0].scatter(spring_rates / 175.12683464567, Fr_Freq_max, c="k")
axs[2, 0].scatter(spring_rates / 175.12683464567, Rr_Freq_max, c="m")
axs[2, 0].legend()

# Max displacement plotting
axs[2, 1].plot(spring_rates / 175.12683464567, max_Fr_corner_z_max, c="k", label="Front Corner")
axs[2, 1].plot(spring_rates / 175.12683464567, max_Rr_corner_z_max, c="m", label="Rear Corner")
axs[2, 1].plot(spring_rates / 175.12683464567, [ride_height for _ in range(len(spring_rates))], c="r")
axs[2, 1].scatter(spring_rates / 175.12683464567, max_Fr_corner_z_max, c="k")
axs[2, 1].scatter(spring_rates / 175.12683464567, max_Rr_corner_z_max, c="m")
axs[2, 1].legend()

plt.show()
# print(Fr_Freq)
# print(Rr_Freq)