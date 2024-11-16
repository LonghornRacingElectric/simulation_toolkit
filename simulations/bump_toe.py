from vehicle_model.vehicle_model import VehicleModel

def calc():
    # Inboard tie z coord
    inb_z_coord = 0.1456182

    # Outboard tie x and y coords
    out_x_coord = -1.45744993
    out_y_coord = 0.58648783

    new_vehicle = VehicleModel(defn_file_path="model_inputs/Nightwatch.yml", 
                            tir_file_path="model_inputs/Modified_Round_8_Hoosier_R25B_16x7p5_10_on_7in_12psi_PAC02_UM2.tir",
                            aero_map_path='model_inputs/aero_map.csv')

    # [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    inboard_points = [
        [-1.28834, 0.260, 0.245],
        [-1.50424, 0.260, 0.245],
        [-1.28834, 0.25205, 0.08],
        [-1.50424, 0.25205, 0.08],
        [-1.390447, 0.3097022, 0.1456182],
        [-1.47573900, 0.30957935, 0.11471471]]

    # [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    outboard_points = [
        [-1.5501596, 0.575, 0.2952369],
        [-1.5501596, 0.575, 0.2952369],
        [-1.554986, 0.5775969, 0.10541],
        [-1.554986, 0.5775969, 0.10541],
        [-1.45744993, 0.58648783, 0.2005],
        [-1.55040452, 0.51228235, 0.29667004]]

    IUFyz = inboard_points[0][1:]
    ILFyz = inboard_points[2][1:]

    ILFxz = [inboard_points[2][0]] + [inboard_points[2][2]]
    IUAxz = [inboard_points[1][0]] + [inboard_points[1][2]]

    y_of_z = lambda z: (ILFyz[0] - IUFyz[0]) / (ILFyz[1] - IUFyz[1]) * (z - ILFyz[1]) + ILFyz[0]
    x_of_z = lambda z: (ILFxz[0] - IUAxz[0]) / (ILFxz[1] - IUAxz[1]) * (z - ILFxz[1]) + ILFxz[0]

    x_coord = x_of_z(inb_z_coord)
    y_coord = y_of_z(inb_z_coord)
    inboard_pos = (x_coord, y_coord, inb_z_coord)

    inboard_pos_yz = inboard_pos[1:]
    FVIC_position_yz = new_vehicle.suspension.FL_double_wishbone.FVIC.position[1:]

    z_of_y = lambda y: float((FVIC_position_yz[1] - inboard_pos_yz[1]) / (FVIC_position_yz[0] - inboard_pos_yz[0]) * (y - inboard_pos_yz[0]) + inboard_pos_yz[1])

    out_z_coord = z_of_y(out_y_coord)
    outboard_pos = (out_x_coord, out_y_coord, out_z_coord)

    print(f"Inboard Tie Position: {inboard_pos}")
    print(f"Outboard Tie Position: {outboard_pos}")