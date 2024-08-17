from suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from suspension_model.suspension_elements.quaternary_elements.axle import Axle
from suspension_model.suspension_elements.secondary_elements.cg import CG
from suspension_model.assets.plotter import Plotter
from typing import Callable
from typing import Sequence
from typing import Tuple
import pyvista as pv
import numpy as np
import time


class SuspensionModel:
    def __init__(
            self,

            FL_inboard_points: Sequence[Sequence[float]],
            FL_outboard_points: Sequence[Sequence[float]],
            FL_contact_patch: Sequence[float],
            FL_inclination_angle: float,
            FL_toe: float,

            FR_inboard_points: Sequence[Sequence[float]],
            FR_outboard_points: Sequence[Sequence[float]],
            FR_contact_patch: Sequence[float],
            FR_inclination_angle: float,
            FR_toe: float,

            RL_inboard_points: Sequence[Sequence[float]],
            RL_outboard_points: Sequence[Sequence[float]],
            RL_contact_patch: Sequence[float],
            RL_inclination_angle: float,
            RL_toe: float,

            RR_inboard_points: Sequence[Sequence[float]],
            RR_outboard_points: Sequence[Sequence[float]],
            RR_contact_patch: Sequence[float],
            RR_inclination_angle: float,
            RR_toe: float,

            tire_radius: float,
            tire_width: float,

            cg_location: Sequence[float],
            show_ICs: bool,
            
            plotter: pv.Plotter) -> None:
        
        # Initialize plotter
        self.plotter = plotter
        self.verbose: bool = False

        # Initialize each corner assembly
        self.FL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FL_inboard_points,
                                                                outboard_points=FL_outboard_points,
                                                                contact_patch=FL_contact_patch,
                                                                inclination_angle=FL_inclination_angle,
                                                                toe=FL_toe,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.FR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FR_inboard_points,
                                                                outboard_points=FR_outboard_points,
                                                                contact_patch=FR_contact_patch,
                                                                inclination_angle=FR_inclination_angle,
                                                                toe=FR_toe,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RL_inboard_points,
                                                                outboard_points=RL_outboard_points,
                                                                contact_patch=RL_contact_patch,
                                                                inclination_angle=RL_inclination_angle,
                                                                toe=RL_toe,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RR_inboard_points,
                                                                outboard_points=RR_outboard_points,
                                                                contact_patch=RR_contact_patch,
                                                                inclination_angle=RR_inclination_angle,
                                                                toe=RR_toe,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
        
        # CG location
        self.cg: CG = CG(position=cg_location)

        # Initialize each axle
        self.Fr_axle: Axle = Axle(left_assy=self.FL_double_wishbone, right_assy=self.FR_double_wishbone, cg=self.cg)
        self.Rr_axle: Axle = Axle(left_assy=self.RL_double_wishbone, right_assy=self.RR_double_wishbone, cg=self.cg)
        
        # Initialize front and rear axles together
        self.full_suspension: FullSuspension = FullSuspension(Fr_axle=self.Fr_axle, Rr_axle=self.Rr_axle, cg=self.cg)

        # Elements for plotting
        self.elements = [self.full_suspension]
    
    def steer(self, rack_displacement: float):
        self.Fr_axle.steer(rack_displacement=rack_displacement)
    
    def jounce(self, jounce: float):
        self.Fr_axle.axle_jounce(jounce=jounce)
        self.Rr_axle.axle_jounce(jounce=jounce)
    
    def roll(self, roll: float):
        start = time.time()
        self.Fr_axle.roll(angle=roll)
        self.Rr_axle.roll(angle=roll)
        end = time.time()

        print(end - start)

    def plot_elements(self, plotter, verbose):
        self.verbose = verbose
        plotter.clear()
        plotter.add_ground(FL_cp=self.FL_double_wishbone.contact_patch, RL_cp=self.RL_double_wishbone.contact_patch, tire=self.FL_double_wishbone.tire)
        for element in self.elements:
            element.plot_elements(plotter=plotter, verbose=self.verbose)

    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]):
        self.plotter.add_slider(func, title, bounds, pos)

    def steer_slider(self, steer: float):
        self.steer(rack_displacement=steer)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def jounce_slider(self, jounce: float):
        self.jounce(jounce=jounce)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def roll_slider(self, roll: float):
        self.roll(roll=roll * np.pi / 180)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)