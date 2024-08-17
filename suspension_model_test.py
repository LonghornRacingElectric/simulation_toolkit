from suspension_model.assets.plotter import Plotter
from suspension_model.suspension_model import SuspensionModel
import time as time
import numpy as np

# CG defn
cg_location = [-40, 0, 11]

# FL defn
FL_inboard_points = [[4.42593860, 10.00000000, 8.45324232], 
                  [-5.12500000, 10.00000000, 7.25937500],
                  [4.42593860, 7.75000000, 2.89999228],
                  [-5.12500000, 7.75000000, 2.90000894],
                  [2.25000000, 7.75000000, 4.13100000]]
FL_outboard_points = [[0.00000000, 22.85030209, 11.55504257],
                   [0.00000000, 22.85030209, 11.55504257], 
                   [0.00000000, 23.39566598, 4.20504257], 
                   [0.00000000, 23.39566598, 4.20504257],
                   [2.18237215, 22.75465155, 6.10552788]]
FL_contact_patch = [0.00000000, 24.93900000, 0.00000000]
FL_inclination_angle = 1 * np.pi / 180
FL_toe = 0 * np.pi / 180

# FR defn
FR_inboard_points = [[p[0], -1 * p[1], p[2]] for p in FL_inboard_points]
FR_outboard_points = [[p[0], -1 * p[1], p[2]] for p in FL_outboard_points]
FR_contact_patch = [FL_contact_patch[0], -1 * FL_contact_patch[1], FL_contact_patch[2]]
FR_inclination_angle = -1 * np.pi / 180
FR_toe = 0 * np.pi / 180

# RL defn
RL_inboard_points = [[-55.875, 11.800, 8.580], 
                  [-69.150, 11.800, 8.580],
                  [-55.875, 10.500, 2.900],
                  [-67.250, 10.500, 2.900],
                  [-55.517, 11.226, 5.716]]
RL_outboard_points = [[-61.131, 21.421, 11.653],
                   [-61.131, 21.421, 11.653], 
                   [-60.861, 22.846, 4.053], 
                   [-60.861, 22.846, 4.053],
                   [-57.502, 21.983, 8.077]]
RL_contact_patch = [-61.000, 24.000, 0.000]
RL_inclination_angle = 1 * np.pi / 180
RL_toe = 0 * np.pi / 180

# RR defn
RR_inboard_points = [[p[0], -1 * p[1], p[2]] for p in RL_inboard_points]
RR_outboard_points = [[p[0], -1 * p[1], p[2]] for p in RL_outboard_points]
RR_contact_patch = [RL_contact_patch[0], -1 * RL_contact_patch[1], RL_contact_patch[2]]
RR_inclination_angle = -1 * np.pi / 180
RR_toe = 0 * np.pi / 180

tire_radius = 8
tire_width = 7

plotter = Plotter()

suspension_model = SuspensionModel(FL_inboard_points=FL_inboard_points,
                                   FL_outboard_points=FL_outboard_points,
                                   FL_contact_patch=FL_contact_patch,
                                   FL_inclination_angle=FL_inclination_angle,
                                   FL_toe=FL_toe,

                                   FR_inboard_points=FR_inboard_points,
                                   FR_outboard_points=FR_outboard_points,
                                   FR_contact_patch=FR_contact_patch,
                                   FR_inclination_angle=FR_inclination_angle,
                                   FR_toe=FR_toe,
                                   
                                   RL_inboard_points=RL_inboard_points,
                                   RL_outboard_points=RL_outboard_points,
                                   RL_contact_patch=RL_contact_patch,
                                   RL_inclination_angle=RL_inclination_angle,
                                   RL_toe=RL_toe,

                                   RR_inboard_points=RR_inboard_points,
                                   RR_outboard_points=RR_outboard_points,
                                   RR_contact_patch=RR_contact_patch,
                                   RR_inclination_angle=RR_inclination_angle,
                                   RR_toe=RR_toe,

                                   tire_radius=tire_radius,
                                   tire_width=tire_width,

                                   cg_location=cg_location,
                                   show_ICs=False,
                                   
                                   plotter=plotter)

jounce_vals = list(np.linspace(0, 3, 10)) + list(np.linspace(3, 0, 10)) + list(np.linspace(0, -3, 10)) + list(np.linspace(-3, 0, 10))
steer_vals = list(np.linspace(0, 1, 10)) + list(np.linspace(1, 0, 10)) + list(np.linspace(0, -1, 10)) + list(np.linspace(-1, 0, 10))

suspension_model.plot_elements(plotter=plotter, verbose=True, show_grid=False)

# suspension_model.plot_links()
# suspension_model.plot_cp()
# suspension_model.plot_tire()

# suspension_model.start_gif()
# start = time.time()
# for jounce in jounce_vals:
#     suspension_model.jounce(jounce)
    # suspension_model.plot_links()
    # suspension_model.plot_cp()
    # suspension_model.plot_tire()
    # suspension_model.add_frame()

# for steer in steer_vals:
#     suspension_model.steer(steer)
#     suspension_model.plot_links()
#     suspension_model.plot_cp()
#     suspension_model.plot_tire()
#     suspension_model.add_frame()
# end = time.time()

# suspension_model.end_gif()

# print(end - start)

plotter.add_slider(func=suspension_model.roll_slider, title="Roll", bounds=[-10, 10], pos=[[0.75, 0.1], [0.98, 0.1]])
plotter.add_slider(func=suspension_model.pitch_slider, title="Pitch", bounds=[-10, 10], pos=[[0.75, 0.225], [0.98, 0.225]])
plotter.add_slider(func=suspension_model.heave_slider, title="Heave", bounds=[-10, 10], pos=[[0.75, 0.350], [0.98, 0.350]])
plotter.add_slider(func=suspension_model.steer_slider, title="Steer", bounds=[-2, 2], pos=[[0.75, 0.475], [0.98, 0.475]])

plotter.show()