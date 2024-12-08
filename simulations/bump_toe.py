from vehicle_model.vehicle_model import VehicleModel

def calc():
    # Inboard tie z coord
    inb_z_coord = 0.11684

    # Outboard tie x and y coords
    out_x_coord = 0.05537382
    out_y_coord = 0.570

    new_vehicle = VehicleModel(defn_file_path="model_inputs/Nightwatch.yml", 
                            tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir",
                            aero_map_path='model_inputs/aero_map.csv')

    # [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    inboard_points = [
      [0.11226939, 0.2159, 0.2],
      [-0.11226939, 0.2159, 0.2],
      [0.112776, 0.2159, 0.09],
      [-0.09525, 0.2159, 0.09],
      [0.07185865636363636, 0.2159, 0.11],
      [0, 0.27638715, 0.08312027]]

    # [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    outboard_points = [
      [-0.00634805, 0.52578, 0.295],
      [-0.00634805, 0.52578, 0.295],
      [0, 0.5565, 0.125],
      [0, 0.5565, 0.125],
      [0.05537382, 0.57, 0.15948244582625945],
      [-0.00002554, 0.51228060, 0.25244758]]

    IUFyz = inboard_points[0]
    ILFyz = inboard_points[2]

    ILFxz = inboard_points[2]
    IUAxz = inboard_points[1]

    y_of_z = lambda z: (ILFyz[1] - IUFyz[1]) / (ILFyz[2] - IUFyz[2]) * (z - ILFyz[2]) + ILFyz[1]
    x_of_z = lambda z: (ILFxz[0] - IUAxz[0]) / (ILFxz[2] - IUAxz[2]) * (z - ILFxz[2]) + ILFxz[0]

    x_coord = x_of_z(inb_z_coord)
    y_coord = y_of_z(inb_z_coord)
    inboard_pos = (x_coord, y_coord, inb_z_coord)

    new_vehicle.suspension.full_suspension.hard_reset()
    FVIC_position_yz = new_vehicle.suspension.FL_double_wishbone.FVIC.position

    z_of_y = lambda y: float((FVIC_position_yz[2] - inboard_pos[2]) / (FVIC_position_yz[1] - inboard_pos[1]) * (y - inboard_pos[1]) + inboard_pos[2])

    out_z_coord = z_of_y(out_y_coord)
    outboard_pos = (out_x_coord, out_y_coord, out_z_coord)

    print(f"Inboard Tie Position: {inboard_pos}")
    print(f"Outboard Tie Position: {outboard_pos}")