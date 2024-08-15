from suspension_model.suspension_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.link import Link
from suspension_model.suspension_elements.node import Node
from suspension_model.assets.plotter import Plotter
from typing import Callable
from typing import Sequence
from typing import Tuple
import pyvista as pv


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
        
        # Kin RC shit as a proof of concept
        self.Fr_KinRC: Node = Node(position=self.calc_Fr_KinRC())
        self.Rr_KinRC: Node = Node(position=self.calc_Rr_KinRC())
        self.roll_axis: Link = Link(inboard=self.Fr_KinRC, outboard=self.Rr_KinRC)
        
        self.elements = [self.FL_double_wishbone, self.FR_double_wishbone, self.RL_double_wishbone, self.RR_double_wishbone]
        self.hidden_elements = [self.Fr_KinRC, self.Rr_KinRC, self.roll_axis]

    def calc_Fr_KinRC(self):
        return self.FL_double_wishbone.FVIC_link.yz_intersection(link=self.FR_double_wishbone.FVIC_link)
    
    def calc_Rr_KinRC(self):
        return self.RL_double_wishbone.FVIC_link.yz_intersection(link=self.RR_double_wishbone.FVIC_link)
    
    def steer(self, rack_displacement: float):
        self.FL_double_wishbone.steer(rack_displacement)
        self.FR_double_wishbone.steer(rack_displacement)
        
        self.Fr_KinRC.position = self.calc_Fr_KinRC()
        self.Rr_KinRC.position = self.calc_Rr_KinRC()
    
    def jounce(self, jounce: float):
        self.FL_double_wishbone.jounce(jounce)
        self.FR_double_wishbone.jounce(-1 * jounce)
        self.RL_double_wishbone.jounce(jounce)
        self.RR_double_wishbone.jounce(-1 * jounce)

        self.Fr_KinRC.position = self.calc_Fr_KinRC()
        self.Rr_KinRC.position = self.calc_Rr_KinRC()

    def plot_elements(self, plotter, verbose):
        self.verbose = verbose
        plotter.clear()
        for corner in self.elements:
            corner.plot_elements(plotter=plotter)
        
        if verbose:
            for element in self.hidden_elements:
                element.plot_elements(plotter=plotter)

    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]):
        self.plotter.add_slider(func, title, bounds, pos)

    def steer_slider(self, steer: float):
        self.steer(rack_displacement=steer)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def jounce_slider(self, jounce: float):
        self.jounce(jounce=jounce)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)
