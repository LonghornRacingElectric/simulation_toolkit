from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.assets.plotter import Plotter
import time as time
import numpy as np

inboard_points = [[4.42593860, 10.00000000, 8.45324232], 
                  [-5.12500000, 10.00000000, 7.25937500],
                  [4.42593860, 7.75000000, 2.89999228],
                  [-5.12500000, 7.75000000, 2.90000894],
                  [2.25000000, 7.75000000, 4.13100000]]
outboard_points = [[0.00000000, 22.85030209, 11.55504257],
                   [0.00000000, 22.85030209, 11.55504257], 
                   [0.00000000, 23.39566598, 4.20504257], 
                   [0.00000000, 23.39566598, 4.20504257],
                   [2.18237215, 22.75465155, 6.10552788]]
inclination_angle = 1 * np.pi / 180
toe = 0 * np.pi / 180
tire_radius = 8
tire_width = 7
contact_patch = [0.00000000, 24.93900000, 0.00000000]

corner = DoubleWishbone(
    inboard_points=inboard_points, 
    outboard_points=outboard_points, 
    contact_patch=contact_patch, 
    inclination_angle=inclination_angle,
    toe=toe,
    tire_radius=tire_radius, 
    tire_width=tire_width,
    show_ICs=False)

plotter = Plotter()

corner.steer(0)
corner.jounce(0)
corner.plot_elements(plotter=plotter)
plotter.show()