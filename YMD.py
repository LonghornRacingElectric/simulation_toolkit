from vehicle_model.vehicle_model import VehicleModel
from simulations.yaw_moment import YMD

new_vehicle = VehicleModel(defn_file_path="model_inputs/Nightwatch.yml", 
                           tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir",
                           aero_map_path='model_inputs/aero_map.csv')

ymd_generator = YMD(vehicle=new_vehicle, mesh=21)

# ymd_generator.generate_constant_velocity_YMD(velocity=5)
# ymd_generator.generate_constant_velocity_YMD(velocity=10)
# ymd_generator.generate_constant_velocity_YMD(velocity=15)
# ymd_generator.generate_constant_velocity_YMD(velocity=20)
# ymd_generator.generate_constant_velocity_YMD(velocity=25)
# ymd_generator.generate_constant_velocity_YMD(velocity=30)
# ymd_generator.generate_constant_velocity_YMD(velocity=35)
# ymd_generator.generate_constant_velocity_YMD(velocity=40)
ymd_generator.generate_constant_radius_YMD(radius=1000)