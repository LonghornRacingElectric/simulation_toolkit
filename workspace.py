from simulations.kin_optimization import KinOptimization
from simulations.optimal_tire_state import OptimalTire
from vehicle_model.vehicle_model import VehicleModel
from LHR_tire_toolkit.MF52 import MF52
import numpy as np

vehicle = VehicleModel(defn_file_path="model_inputs/sample_vehicle.yml", tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")

# optim_tire = OptimalTire(vehicle=vehicle)

# optim_tire.optimal_state()

# tire = MF52(tire_name="tire", file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir")
# loads = tire.tire_eval(FZ=tire.get_FNOMIN(), alpha=0, kappa=0.15, gamma=5 * np.pi / 180)
# print([float(x) for x in loads])

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