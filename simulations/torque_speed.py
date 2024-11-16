from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

# tire = MF52(tire_name="tire", file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")

# # files = ["./model_inputs/tire_states_20_mps.csv",
# #          "./model_inputs/tire_states_25_mps.csv",
# #          "./model_inputs/tire_states_30_mps.csv",
# #          "./model_inputs/tire_states_35_mps.csv",
# #          "./model_inputs/tire_states_40_mps.csv"]

# files = ["./model_inputs/tire_states_500_m.csv"]

# fig = plt.figure()
# ax = fig.gca()

# vel = []
# peak_F_per_vel = []

# for index, file in enumerate(files):
#     df = pd.read_csv(file)

#     FL_Fz_lst = df["FL_Fz"].tolist()
#     FR_Fz_lst = df["FR_Fz"].tolist()
#     RL_Fz_lst = df["RL_Fz"].tolist()
#     RR_Fz_lst = df["RR_Fz"].tolist()
    
#     FL_SA_lst = df["FL_SA"].tolist()
#     FR_SA_lst = df["FR_SA"].tolist()
#     RL_SA_lst = df["RL_SA"].tolist()
#     RR_SA_lst = df["RR_SA"].tolist()

#     FL_gamma_lst = df["FL gamma"].tolist()
#     FR_gamma_lst = df["FR gamma"].tolist()
#     RL_gamma_lst = df["RL gamma"].tolist()
#     RR_gamma_lst = df["RR gamma"].tolist()

#     velocity_lst = df["Velocity"].tolist()
    
#     peak_Fx_rear_axle = []

#     h_cg = 11 * 0.0254
#     l = 61 * 0.0254
#     m = 240

#     for i in range(len(FL_Fz_lst)):

#         FL_peak_Fx = tire.get_mu(FZ=FL_Fz_lst[i], alpha=FL_SA_lst[i], kappa=0, gamma=FL_gamma_lst[i])[0] * FL_Fz_lst[i]
#         FR_peak_Fx = tire.get_mu(FZ=FR_Fz_lst[i], alpha=FR_SA_lst[i], kappa=0, gamma=FR_gamma_lst[i])[0] * FR_Fz_lst[i]
#         RL_peak_Fx = tire.get_mu(FZ=RL_Fz_lst[i], alpha=RL_SA_lst[i], kappa=0, gamma=RL_gamma_lst[i])[0] * RL_Fz_lst[i]
#         RR_peak_Fx = tire.get_mu(FZ=RR_Fz_lst[i], alpha=RR_SA_lst[i], kappa=0, gamma=RR_gamma_lst[i])[0] * RR_Fz_lst[i]

#         rear_axle_force = RL_peak_Fx + RR_peak_Fx
#         a_x = rear_axle_force / m

#         dW = h_cg / l * m * a_x

#         new_rear_Fz_per = (rear_axle_force + dW) / 2
#         RL_peak_Fx = tire.get_mu(FZ=new_rear_Fz_per, alpha=RL_SA_lst[i], kappa=0, gamma=RL_gamma_lst[i])[0] * new_rear_Fz_per
#         RR_peak_Fx = RL_peak_Fx
#         rear_Fx = RL_peak_Fx * 2

#         previous_value = rear_Fx
#         delta = 1e9

#         while delta > 1:
#             rear_axle_force = RL_peak_Fx + RR_peak_Fx
#             a_x = rear_axle_force / m

#             dW = h_cg / l * m * a_x

#             new_rear_Fz_per = (rear_axle_force + dW) / 2
#             RL_peak_Fx = tire.get_mu(FZ=new_rear_Fz_per, alpha=RL_SA_lst[i], kappa=0, gamma=RL_gamma_lst[i])[0] * new_rear_Fz_per
#             RR_peak_Fx = RL_peak_Fx
#             rear_Fx = RL_peak_Fx * 2
#             delta = previous_value - rear_Fx
#             previous_value = rear_Fx

#         peak_Fx_rear_axle.append(rear_Fx)

#         vel.append(velocity_lst[i] / (8 * 0.0254) * 30 / np.pi)

# simmons_dict = {"Wheel RPM": vel,
#                 "Peak Axle Fx": peak_Fx_rear_axle}

# df = pd.DataFrame().from_dict(simmons_dict)
# df.to_csv("./outputs/simmons_junk_mess_brother_type_shit.csv")

# ax.set_xlabel("Vehicle-Centered Wheel Speed (RPM)")
# ax.set_ylabel("Peak Possible Fx (N)")
# ax.set_title("Potential Fx vs Wheel Speed")

# ax.scatter(vel, peak_Fx_rear_axle, c='k', label="Peak Axle Force")
# # for (x, y) in zip([round(v) for v in vel], [round(F) for F in peak_F_per_vel]):
# #     plt.text(x, y, f'({x},{y})', fontsize=12, ha='left', va='top')

# ax.grid()
# ax.legend()

# plt.show()

df = pd.read_csv("./model_inputs/simmons_junk_mess_brother_type_shit.csv")

rpm = df["Wheel RPM"].tolist()
peak_Fx = df["Peak Axle Fx"].tolist()

fig = plt.figure()
ax = fig.gca()

ax.set_title("Peak Axle Fx vs RPM")
ax.set_xlabel("Axle Velocity (rpm)")
ax.set_ylabel("Peak Axle Fx (N)")
ax.scatter(rpm, peak_Fx, c='k', s=5)
ax.grid()
fig.savefig("./outputs/simmons.png")