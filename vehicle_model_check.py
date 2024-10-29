from vehicle_model.vehicle_model import VehicleModel

new_vehicle = VehicleModel(defn_file_path="model_inputs/sample_vehicle.yml", 
                           tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir",
                           aero_map_path='model_inputs/aero_map.csv')