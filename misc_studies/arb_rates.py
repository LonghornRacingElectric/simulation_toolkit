import numpy as np

# Again, a glorified calculator
# 1:0.75, 1:1, 1:1.25, 1:2
LLTD = [0.7217557414911843, 0.6604960014353762, 0.608821654874703, 0.4930899811744839] # Forward bias
limit_stability = [-2.3102799182524154, -0.3634952794120033, 1.1647412225924536, 3.334466098457439] # rad/s^2

Fr_track = 48 * 0.0254 # m
Rr_track = 48 * 0.0254 # m

Ks_Fr = 350 * 175.12683464567 # N/m
Ks_Rr = 600 * 175.12683464567 # N/m

# Nominal MR
Fr_MR_nom = 1.273846638
Rr_MR_nom = 1.776764199

Kw_Fr = Ks_Fr / Fr_MR_nom**2
Kw_Rr = Ks_Rr / Rr_MR_nom**2

sprung_mass = 238 - 16.328 - 18.611
print(1 / (2 * np.pi) * np.sqrt((1/Kw_Fr + 1/98947)**-1 / (sprung_mass / 4)))
print(1 / (2 * np.pi) * np.sqrt((1/Kw_Rr + 1/98947)**-1 / (sprung_mass / 4)))

Kr_Fr = 1/2 * Kw_Fr * Fr_track**2
Kr_Rr = 1/2 * Kw_Rr * Rr_track**2

min_ARB_rate = (1 - 0.45) * Kr_Fr / 0.45 - Kr_Rr
nom_ARB_rate = (1 - 0.50) * Kr_Fr / 0.50 - Kr_Rr
max_ARB_rate = (1 - 0.55) * Kr_Fr / 0.55 - Kr_Rr

ARB_rate = (max_ARB_rate + min_ARB_rate) / 2
ARB_rate_range = (min_ARB_rate - max_ARB_rate) / 2

print(f"ARB rate: {ARB_rate} +/- {ARB_rate_range} Nm/rad")

print(f"Min ARB Kr: {min_ARB_rate} Nm/rad")
print(f"Nom ARB Kr: {nom_ARB_rate} Nm/rad")
# print(f"Max ARB Kr: {max_ARB_rate} Nm/rad")

Kr_Rr += 3336.1516153964585
Kr_Rr += 9574.847999415106
# print(Kr_Fr)
# print(Kr_Rr)
print(Kr_Fr / (Kr_Fr + Kr_Rr))