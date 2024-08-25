from simulations.kin_optimization import KinOptimization
from vehicle_model.vehicle_model import VehicleModel
import numpy as np

vehicle = VehicleModel(defn_file_path="model_inputs/sample_vehicle.yml", tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")

# print(np.linspace(-5 * 0.0254, 5 * 0.0254, 5))
# print(np.linspace(-3, 3, 5))
# print(np.linspace(-3, 3, 5))


kin_optim = KinOptimization(vehicle_model=vehicle,
                            steer_sweep=np.linspace(-1.5 * 0.0254, 1.5 * 0.0254, 6),
                            heave_sweep=np.linspace(-2 * 0.0254, 2 * 0.0254, 6), 
                            pitch_sweep=np.linspace(-2, 2, 6),
                            roll_sweep=np.linspace(-2, 2, 6))

kin_optim.optimize()
# kin_optim.plot()