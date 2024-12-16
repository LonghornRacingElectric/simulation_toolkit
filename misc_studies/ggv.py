from LHR_tire_toolkit.MF52 import MF52
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

tire = MF52(tire_name="tire", file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")

# ymd_5_mps = pd.read_csv("./model_inputs/tire_states_5_mps.csv")
ymd_10_mps = pd.read_csv("./model_inputs/tire_states_10_mps.csv")
ymd_15_mps = pd.read_csv("./model_inputs/tire_states_15_mps.csv")
ymd_20_mps = pd.read_csv("./model_inputs/tire_states_20_mps.csv")
ymd_25_mps = pd.read_csv("./model_inputs/tire_states_25_mps.csv")
ymd_30_mps = pd.read_csv("./model_inputs/tire_states_30_mps.csv")
ymd_35_mps = pd.read_csv("./model_inputs/tire_states_35_mps.csv")
ymd_40_mps = pd.read_csv("./model_inputs/tire_states_40_mps.csv")
ymd_250_m = pd.read_csv("./model_inputs/tire_states_250_m.csv")
ymd_500_m = pd.read_csv("./model_inputs/tire_states_500_m.csv")
ymd_1000_m = pd.read_csv("./model_inputs/tire_states_1000_m.csv")
ymd_1500_m = pd.read_csv("./model_inputs/tire_states_1500_m.csv")
ymd_250s_m = pd.read_csv("./model_inputs/tire_states_250s_m.csv")
ymd_500s_m = pd.read_csv("./model_inputs/tire_states_500s_m.csv")
ymd_1000s_m = pd.read_csv("./model_inputs/tire_states_1000s_m.csv")
ymd_1500s_m = pd.read_csv("./model_inputs/tire_states_1500s_m.csv")

all_dfs = [ymd_10_mps, 
           ymd_15_mps, 
           ymd_20_mps, 
           ymd_25_mps, 
           ymd_30_mps, 
           ymd_35_mps, 
           ymd_40_mps,
           ymd_250_m,
           ymd_500_m,
           ymd_1000_m,
           ymd_1500_m,
           ymd_250s_m,
           ymd_500s_m,
           ymd_1000s_m,
           ymd_1500s_m]

Fz = []
Fx = []
Fy = []
alpha = []
gamma = []

# vel = []
for output in all_dfs:
    for i in range(len(output["FL_Fz"].tolist())):
        Fz.append(output["FL_Fz"].tolist()[i])
        Fz.append(output["FR_Fz"].tolist()[i])
        Fz.append(output["RL_Fz"].tolist()[i])
        Fz.append(output["RR_Fz"].tolist()[i])

        Fy.append(float(output["FL_Fy"].tolist()[i]))
        Fy.append(float(output["FR_Fy"].tolist()[i]))
        Fy.append(float(output["RL_Fy"].tolist()[i]))
        Fy.append(float(output["RR_Fy"].tolist()[i]))

        Fx.append(float(output["FL_Fx"].tolist()[i]))
        Fx.append(float(output["FR_Fx"].tolist()[i]))
        Fx.append(float(output["RL_Fx"].tolist()[i]))
        Fx.append(float(output["RR_Fx"].tolist()[i]))

        alpha.append(output["FL_SA"].tolist()[i])
        alpha.append(output["FR_SA"].tolist()[i])
        alpha.append(output["RL_SA"].tolist()[i])
        alpha.append(output["RR_SA"].tolist()[i])

        gamma.append(output["FL gamma"].tolist()[i])
        gamma.append(output["FR gamma"].tolist()[i])
        gamma.append(output["RL gamma"].tolist()[i])
        gamma.append(output["RR gamma"].tolist()[i])

    h_cg = 11 * 0.0254
    l = 61 * 0.0254
    m = 274.923547401

new_Fx_Fr = []
new_Fx_Rr = []
new_Fy_Fr = []
new_Fy_Rr = []
new_Fz_Fr = []
new_Fz_Rr = []

# for output in all_dfs:
#     for i in range(len(output["FL_Fz"].tolist())):
#         new_Fz_Fr.append(float(output["FL_Fz"].tolist()[i]))
#         new_Fz_Fr.append(float(output["FR_Fz"].tolist()[i]))
#         new_Fz_Rr.append(float(output["RL_Fz"].tolist()[i]))
#         new_Fz_Rr.append(float(output["RR_Fz"].tolist()[i]))

#         new_Fy_Fr.append(float(output["FL_Fy"].tolist()[i]))
#         new_Fy_Fr.append(float(output["FR_Fy"].tolist()[i]))
#         new_Fy_Rr.append(float(output["RL_Fy"].tolist()[i]))
#         new_Fy_Rr.append(float(output["RR_Fy"].tolist()[i]))

#         new_Fx_Fr.append(float(output["FL_Fx"].tolist()[i]))
#         new_Fx_Fr.append(float(output["FR_Fx"].tolist()[i]))
#         new_Fx_Rr.append(float(output["RL_Fx"].tolist()[i]))
#         new_Fx_Rr.append(float(output["RR_Fx"].tolist()[i]))

for i in range(int(len(Fz) / 4)):
    FL_Fz = Fz[4*i + 0]
    FR_Fz = Fz[4*i + 1]
    RL_Fz = Fz[4*i + 2]
    RR_Fz = Fz[4*i + 3]

    FL_peak_Fx = 0.6 * tire.get_mu(FZ=FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = 0.6 * tire.get_mu(FZ=FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = 0.6 * tire.get_mu(FZ=RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = 0.6 * tire.get_mu(FZ=RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
    a_x = total_Fx / m

    dW = h_cg / l * m * a_x

    FL_Fz -= dW / 2
    FR_Fz -= dW / 2
    RL_Fz += dW / 2
    RR_Fz += dW / 2

    FL_peak_Fx = 0.6 * tire.get_mu(FZ=FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = 0.6 * tire.get_mu(FZ=FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = 0.6 * tire.get_mu(FZ=RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = 0.6 * tire.get_mu(FZ=RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    FL_Fz += dW / 2
    FR_Fz += dW / 2
    RL_Fz -= dW / 2
    RR_Fz -= dW / 2

    previous_value = total_Fx
    delta = 1e9

    while delta > 100:
        total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
        a_x = total_Fx / m

        dW = h_cg / l * m * a_x

        new_FL_Fz = FL_Fz - dW / 2
        new_FR_Fz = FR_Fz - dW / 2
        new_RL_Fz = RL_Fz + dW / 2
        new_RR_Fz = RR_Fz + dW / 2

        FL_peak_Fx = 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * new_FL_Fz
        FR_peak_Fx = 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * new_FR_Fz
        RL_peak_Fx = 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * new_RL_Fz
        RR_peak_Fx = 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * new_RR_Fz

        FL_peak_Fy = 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[1] * new_FL_Fz
        FR_peak_Fy = 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[1] * new_FR_Fz
        RL_peak_Fy = 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[1] * new_RL_Fz
        RR_peak_Fy = 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[1] * new_RR_Fz
        
        delta = previous_value - total_Fx
        previous_value = total_Fx

        new_Fx_Fr.append(float(FL_peak_Fx))
        new_Fx_Fr.append(float(FR_peak_Fx))
        new_Fx_Rr.append(float(RL_peak_Fx))
        new_Fx_Rr.append(float(RR_peak_Fx))

        new_Fy_Fr.append(float(FL_peak_Fy))
        new_Fy_Fr.append(float(FR_peak_Fy))
        new_Fy_Rr.append(float(RL_peak_Fy))
        new_Fy_Rr.append(float(RR_peak_Fy))

        new_Fz_Fr.append(float(new_FL_Fz))
        new_Fz_Fr.append(float(new_FR_Fz))
        new_Fz_Rr.append(float(new_RL_Fz))
        new_Fz_Rr.append(float(new_RR_Fz))

for i in range(int(len(Fz) / 4)):
    FL_Fz = Fz[4*i + 0]
    FR_Fz = Fz[4*i + 1]
    RL_Fz = Fz[4*i + 2]
    RR_Fz = Fz[4*i + 3]

    FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
    a_x = total_Fx / m

    dW = h_cg / l * m * a_x

    FL_Fz -= dW / 2
    FR_Fz -= dW / 2
    RL_Fz += dW / 2
    RR_Fz += dW / 2

    FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    FL_Fz += dW / 2
    FR_Fz += dW / 2
    RL_Fz -= dW / 2
    RR_Fz -= dW / 2

    previous_value = total_Fx
    delta = 1e9

    while delta > 100:
        total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
        a_x = total_Fx / m

        dW = h_cg / l * m * a_x

        new_FL_Fz = FL_Fz - dW / 2
        new_FR_Fz = FR_Fz - dW / 2
        new_RL_Fz = RL_Fz + dW / 2
        new_RR_Fz = RR_Fz + dW / 2

        FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[0] * new_FL_Fz
        FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[0] * new_FR_Fz
        RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[0] * new_RL_Fz
        RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[0] * new_RR_Fz

        FL_peak_Fy = 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=alpha[4*i + 0], kappa=0, gamma=gamma[4*i + 0])[1] * new_FL_Fz
        FR_peak_Fy = 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=alpha[4*i + 1], kappa=0, gamma=gamma[4*i + 1])[1] * new_FR_Fz
        RL_peak_Fy = 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=alpha[4*i + 2], kappa=0, gamma=gamma[4*i + 2])[1] * new_RL_Fz
        RR_peak_Fy = 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=alpha[4*i + 3], kappa=0, gamma=gamma[4*i + 3])[1] * new_RR_Fz
        
        delta = previous_value - total_Fx
        previous_value = total_Fx

        new_Fx_Fr.append(float(FL_peak_Fx))
        new_Fx_Fr.append(float(FR_peak_Fx))
        new_Fx_Rr.append(float(RL_peak_Fx))
        new_Fx_Rr.append(float(RR_peak_Fx))

        new_Fy_Fr.append(float(FL_peak_Fy))
        new_Fy_Fr.append(float(FR_peak_Fy))
        new_Fy_Rr.append(float(RL_peak_Fy))
        new_Fy_Rr.append(float(RR_peak_Fy))

        new_Fz_Fr.append(float(new_FL_Fz))
        new_Fz_Fr.append(float(new_FR_Fz))
        new_Fz_Rr.append(float(new_RL_Fz))
        new_Fz_Rr.append(float(new_RR_Fz))

for i in range(int(len(Fz) / 4)):
    FL_Fz = Fz[4*i + 0]
    FR_Fz = Fz[4*i + 1]
    RL_Fz = Fz[4*i + 2]
    RR_Fz = Fz[4*i + 3]

    FL_peak_Fx = 0.6 * tire.get_mu(FZ=FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = 0.6 * tire.get_mu(FZ=FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = 0.6 * tire.get_mu(FZ=RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = 0.6 * tire.get_mu(FZ=RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
    a_x = total_Fx / m

    dW = h_cg / l * m * a_x

    FL_Fz -= dW / 2
    FR_Fz -= dW / 2
    RL_Fz += dW / 2
    RR_Fz += dW / 2

    FL_peak_Fx = 0.6 * tire.get_mu(FZ=FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = 0.6 * tire.get_mu(FZ=FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = 0.6 * tire.get_mu(FZ=RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = 0.6 * tire.get_mu(FZ=RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    FL_Fz += dW / 2
    FR_Fz += dW / 2
    RL_Fz -= dW / 2
    RR_Fz -= dW / 2

    previous_value = total_Fx
    delta = 1e9

    while delta > 100:
        total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
        a_x = total_Fx / m

        dW = h_cg / l * m * a_x

        new_FL_Fz = FL_Fz - dW / 2
        new_FR_Fz = FR_Fz - dW / 2
        new_RL_Fz = RL_Fz + dW / 2
        new_RR_Fz = RR_Fz + dW / 2

        FL_peak_Fx = 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * new_FL_Fz
        FR_peak_Fx = 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * new_FR_Fz
        RL_peak_Fx = 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * new_RL_Fz
        RR_peak_Fx = 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * new_RR_Fz

        FL_peak_Fy = 0
        FR_peak_Fy = 0
        RL_peak_Fy = 0
        RR_peak_Fy = 0
        
        delta = previous_value - total_Fx
        previous_value = total_Fx

        new_Fx_Fr.append(float(FL_peak_Fx))
        new_Fx_Fr.append(float(FR_peak_Fx))
        new_Fx_Rr.append(float(RL_peak_Fx))
        new_Fx_Rr.append(float(RR_peak_Fx))

        new_Fy_Fr.append(float(FL_peak_Fy))
        new_Fy_Fr.append(float(FR_peak_Fy))
        new_Fy_Rr.append(float(RL_peak_Fy))
        new_Fy_Rr.append(float(RR_peak_Fy))

        new_Fz_Fr.append(float(new_FL_Fz))
        new_Fz_Fr.append(float(new_FR_Fz))
        new_Fz_Rr.append(float(new_RL_Fz))
        new_Fz_Rr.append(float(new_RR_Fz))

for i in range(int(len(Fz) / 4)):
    FL_Fz = Fz[4*i + 0]
    FR_Fz = Fz[4*i + 1]
    RL_Fz = Fz[4*i + 2]
    RR_Fz = Fz[4*i + 3]

    FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
    a_x = total_Fx / m

    dW = h_cg / l * m * a_x

    FL_Fz -= dW / 2
    FR_Fz -= dW / 2
    RL_Fz += dW / 2
    RR_Fz += dW / 2

    FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * FL_Fz
    FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * FR_Fz
    RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * RL_Fz
    RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * RR_Fz

    FL_Fz += dW / 2
    FR_Fz += dW / 2
    RL_Fz -= dW / 2
    RR_Fz -= dW / 2

    previous_value = total_Fx
    delta = 1e9

    while delta > 100:
        total_Fx = FL_peak_Fx + FR_peak_Fx + RL_peak_Fx + RR_peak_Fx
        a_x = total_Fx / m

        dW = h_cg / l * m * a_x

        new_FL_Fz = FL_Fz - dW / 2
        new_FR_Fz = FR_Fz - dW / 2
        new_RL_Fz = RL_Fz + dW / 2
        new_RR_Fz = RR_Fz + dW / 2

        FL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_FL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 0])[0] * new_FL_Fz
        FR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_FR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 1])[0] * new_FR_Fz
        RL_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_RL_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 2])[0] * new_RL_Fz
        RR_peak_Fx = -1 * 0.6 * tire.get_mu(FZ=new_RR_Fz, alpha=0, kappa=0, gamma=gamma[4*i + 3])[0] * new_RR_Fz

        FL_peak_Fy = 0
        FR_peak_Fy = 0
        RL_peak_Fy = 0
        RR_peak_Fy = 0
        
        delta = previous_value - total_Fx
        previous_value = total_Fx

        new_Fx_Fr.append(float(FL_peak_Fx))
        new_Fx_Fr.append(float(FR_peak_Fx))
        new_Fx_Rr.append(float(RL_peak_Fx))
        new_Fx_Rr.append(float(RR_peak_Fx))

        new_Fy_Fr.append(float(FL_peak_Fy))
        new_Fy_Fr.append(float(FR_peak_Fy))
        new_Fy_Rr.append(float(RL_peak_Fy))
        new_Fy_Rr.append(float(RR_peak_Fy))

        new_Fz_Fr.append(float(new_FL_Fz))
        new_Fz_Fr.append(float(new_FR_Fz))
        new_Fz_Rr.append(float(new_RL_Fz))
        new_Fz_Rr.append(float(new_RR_Fz))

load_lst_Fr = list([list(x) for x in zip(np.array(new_Fx_Fr), np.array(new_Fy_Fr), np.array(new_Fz_Fr))])
load_lst_Rr = list([list(x) for x in zip(np.array(new_Fx_Rr), np.array(new_Fy_Rr), np.array(new_Fz_Rr))])

accel_case_Rr = sorted(load_lst_Rr, key=lambda x: x[0], reverse=True)[0]
accel_case_Fr = load_lst_Fr[load_lst_Rr.index(accel_case_Rr)]

brake_case_Fr = sorted(load_lst_Fr, key=lambda x: x[0], reverse=False)[0]
brake_case_Rr = load_lst_Rr[load_lst_Fr.index(brake_case_Fr)]

combined_case_accel_Rr = sorted(filter(lambda x: x[0] > 0, load_lst_Rr), key=lambda x: np.sqrt(x[0]**2+x[1]**2+x[2]**2), reverse=True)[0]
combined_case_accel_Fr = load_lst_Fr[load_lst_Rr.index(combined_case_accel_Rr)]

combined_case_brake_Fr = sorted(filter(lambda x: x[0] < 0, load_lst_Fr), key=lambda x: np.sqrt(x[0]**2+x[1]**2+x[2]**2), reverse=True)[0]
combined_case_brake_Rr = load_lst_Rr[load_lst_Fr.index(combined_case_brake_Fr)]

# load_names = ["Pure Acceleration", "Pure Braking", "Combined Cornering Accel", "Combined Cornering Brake"]
load_names = ["Pure Acceleration", "Pure Braking", "Combined Acceleration Left Turn", "Combined Acceleration Right Turn", "Combined Braking Right Turn", "Combined Braking Left Turn"]
loads = [[accel_case_Fr, accel_case_Rr], [brake_case_Fr, brake_case_Rr], [combined_case_accel_Fr, combined_case_accel_Rr], [combined_case_accel_Fr, combined_case_accel_Rr], [combined_case_brake_Fr, combined_case_brake_Rr], [combined_case_brake_Fr, combined_case_brake_Rr]]

readme = ""
for index, load_name in enumerate(load_names):
    if "Left" in load_name:
        if loads[index][0][1] < 0:
            loads[index][0][1] *= -1
        if loads[index][1][1] < 0:
            loads[index][1][1] *= -1
    elif "Right" in load_name:
        if loads[index][0][1] > 0:
            loads[index][0][1] *= -1
        if loads[index][1][1] > 0:
            loads[index][1][1] *= -1

    readme += f"### {load_name}\n"
    readme += "| | Fx (N) | Fy (N) | Fz (N) | Fx (lbf) | Fy (lbf) | Fz (lbf) |\n"
    readme += f"| :---: | :---: | :---: | :---: | :---: | :---: | :---: |\n"
    readme += f"| Front | {round(loads[index][0][0])} | {round(loads[index][0][1])} | {round(loads[index][0][2])} | {round(loads[index][0][0] * 0.2248089431)} | {round(loads[index][0][1] * 0.2248089431)} | {round(loads[index][0][2] * 0.2248089431)} |\n"
    readme += f"| Rear | {round(loads[index][1][0])} | {round(loads[index][1][1])} | {round(loads[index][1][2])} | {round(loads[index][1][0] * 0.2248089431)} | {round(loads[index][1][1] * 0.2248089431)} | {round(loads[index][1][2] * 0.2248089431)} |\n"

with open("./outputs/tire_loads.md", "w") as f:
    f.write(readme)