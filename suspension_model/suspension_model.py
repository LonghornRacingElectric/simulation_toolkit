from suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from suspension_model.suspension_elements.quaternary_elements.axle import Axle
from suspension_model.suspension_elements.secondary_elements.cg import CG
from typing import Callable, Sequence, Tuple
import pyvista as pv
import numpy as np


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
        self.full_suspension.steer(rack_displacement=rack_displacement)
    
    def jounce(self, jounce: float):
        self.Fr_axle.axle_jounce(jounce=jounce)
        self.Rr_axle.axle_jounce(jounce=jounce)
    
    def heave(self, heave: float):
        self.full_suspension.heave(heave=heave)
    
    def pitch(self, pitch: float):
        self.full_suspension.pitch(angle=pitch)
    
    def roll(self, roll: float):
        self.full_suspension.roll(angle=roll)

    def plot_elements(self, plotter, verbose: bool = False, show_grid: bool = False):
        self.verbose = verbose
        plotter.clear()

        if show_grid:
            plotter.show_grid()

        plotter.add_ground(FL_cp=self.FL_double_wishbone.contact_patch, RL_cp=self.RL_double_wishbone.contact_patch, tire=self.FL_double_wishbone.tire)
        for element in self.elements:
            element.plot_elements(plotter=plotter, verbose=self.verbose)

    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]):
        self.plotter.add_slider(func, title, bounds, pos)

    def steer_slider(self, steer: float):
        self.steer(rack_displacement=steer)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)
    
    def heave_slider(self, heave: float):
        self.heave(heave=heave)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def pitch_slider(self, pitch: float):
        self.pitch(pitch=pitch)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def jounce_slider(self, jounce: float):
        self.jounce(jounce=jounce)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def roll_slider(self, roll: float):
        self.roll(roll=roll * np.pi / 180)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)