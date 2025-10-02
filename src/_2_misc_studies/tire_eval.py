from LHR_tire_toolkit.MF52 import MF52

import matplotlib.pyplot as plt
import numpy as np

tire = MF52(tire_name="", file_path="./src/_1_model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")

tire_load = tire.tire_eval(FZ=600, alpha = 5 * np.pi / 180, kappa = 0.15, gamma = 0)

print(tire_load)