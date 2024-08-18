from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from vehicle_model.suspension_model.suspension_elements.quaternary_elements.axle import Axle
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from typing import Callable, Sequence, Tuple
import pyvista as pv
import numpy as np


class SuspensionModel:
    """
    ## Suspension Model

    Designed to model kinematics and force-based properties

    ###### Note, all conventions comply with SAE-J670 Z-up

    Parameters
    ----------
    FL_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the front-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FL_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the front-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FL_contact_patch : Sequence[float]
        Coordinates of the front-left contact patch
    FL_inclination_angle : float
        Inclination angle of the front-left tire in degrees
    FL_toe : float
        Static toe angle of the front-left tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric

    ---

    FR_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the front-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FR_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the front-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FR_contact_patch : Sequence[float]
        Coordinates of the front-right contact patch
    FR_inclination_angle : float
        Inclination angle of the front-right tire in degrees
    FR_toe : float
        Static toe angle of the front-right tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    
    ---

    RL_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the rear-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RL_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the rear-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RL_contact_patch : Sequence[float]
        Coordinates of the rear-left contact patch
    RL_inclination_angle : float
        Inclination angle of the rear-left tire in degrees
    RL_toe : float
        Static toe angle of the rear-left tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    
    --- 

    RR_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the rear-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RR_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the rear-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RR_contact_patch : Sequence[float]
        Coordinates of the rear-right contact patch
    RR_inclination_angle : float
        Inclination angle of the rear-right tire in degrees
    RR_toe : float
        Static toe angle of the rear-right tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    
    ---

    tire_radius : float
        Radius of tire
        - Must use same units as suspension coordinates above
    tire_width : float
        Width of tire
        - Must use same units as suspension coordinates above
    cg_location : Sequence[float]
        Coordinates of the center of gravity
        - Must use same units as suspension coordinates above
    show_ICs : bool
        Toggle visibility of double wishbones vs swing arms
    plotter : pv.Plotter
        Plotting object
        - Import using ```from vehicle_model.suspension_model.assets.plotter import Plotter```
        - Define plotter object using ```plotter = Plotter()```
    """
    def __init__(
            self,

            FL_inboard_points: Sequence[Sequence[float]],
            FL_outboard_points: Sequence[Sequence[float]],
            FL_contact_patch: Sequence[float],
            FL_inclination_angle: float,
            FL_toe: float,
            FL_rate: float,
            FL_MR: float,

            FR_inboard_points: Sequence[Sequence[float]],
            FR_outboard_points: Sequence[Sequence[float]],
            FR_contact_patch: Sequence[float],
            FR_inclination_angle: float,
            FR_toe: float,
            FR_rate: float,
            FR_MR: float,

            Fr_ARBK: float,
            Fr_ARBMR: float,

            RL_inboard_points: Sequence[Sequence[float]],
            RL_outboard_points: Sequence[Sequence[float]],
            RL_contact_patch: Sequence[float],
            RL_inclination_angle: float,
            RL_toe: float,
            RL_rate: float,
            RL_MR: float,

            RR_inboard_points: Sequence[Sequence[float]],
            RR_outboard_points: Sequence[Sequence[float]],
            RR_contact_patch: Sequence[float],
            RR_inclination_angle: float,
            RR_toe: float,
            RR_rate: float,
            RR_MR: float,

            Rr_ARBK: float,
            Rr_ARBMR: float,

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
                                                                inclination_angle=FL_inclination_angle * np.pi / 180,
                                                                toe=FL_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.FR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FR_inboard_points,
                                                                outboard_points=FR_outboard_points,
                                                                contact_patch=FR_contact_patch,
                                                                inclination_angle=FR_inclination_angle * np.pi / 180,
                                                                toe=FR_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RL_inboard_points,
                                                                outboard_points=RL_outboard_points,
                                                                contact_patch=RL_contact_patch,
                                                                inclination_angle=RL_inclination_angle * np.pi / 180,
                                                                toe=RL_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RR_inboard_points,
                                                                outboard_points=RR_outboard_points,
                                                                contact_patch=RR_contact_patch,
                                                                inclination_angle=RR_inclination_angle * np.pi / 180,
                                                                toe=RR_toe * np.pi / 180,
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

        # Rate params
        self.FL_rate = FL_rate
        self.FL_MR = FL_MR
        self.FR_rate = FR_rate
        self.FR_MR = FR_MR
        self.RL_rate = RL_rate
        self.RL_MR = RL_MR
        self.RR_rate = RR_rate
        self.RR_MR = RR_MR
    
    def steer(self, rack_displacement: float) -> None:
        """
        ## Steer

        Steer front suspension axle (capable of supporting rear steer as well)

        Parameters
        ----------
        rack_displacement : float
            Lateral translation of steering rack
            - Must use same units as suspension coordinates

        Returns
        ----------
        None
        """
        self.full_suspension.steer(rack_displacement=rack_displacement)
    
    def heave(self, heave: float) -> None:
        """
        ## Heave

        Heave front and rear suspension axles

        Parameters
        ----------
        heave : float
            Vertical translation of sprung mass
            - Must use same units as suspension coordinates above
            - Down is positive

        Returns
        ----------
        None
        """
        self.full_suspension.heave(heave=heave)
    
    def pitch(self, pitch: float) -> None:
        """
        ## Pitch

        Pitch front and rear suspension axles

        Parameters
        ----------
        pitch : float
            Vehicle pitch in degrees
        
        Returns
        ----------
        None
        """
        self.full_suspension.pitch(angle=pitch * np.pi / 180)
    
    def roll(self, roll: float) -> None:
        """
        ## Roll

        Roll front and rear suspension axles

        Parameters
        ----------
        roll : float
            Vehicle roll in degrees
        
        Returns
        ----------
        None
        """
        self.full_suspension.roll(angle=roll * np.pi / 180)

    def plot_elements(self, plotter: pv.Plotter, verbose: bool = False, show_grid: bool = False) -> None:
        """
        ## Plot Elements

        Plots all children of full suspension model

        Parameters
        ----------
        plotter : py.Plotter
            Plotting object
        verbose : bool, optional
            Toggles visibility of roll/pitch center and cg, by default False
        show_grid : bool, optional
            Toggles grid visibility, by default False
        
        Returns
        ----------
        None
        """
        self.verbose = verbose
        plotter.clear()

        if show_grid:
            plotter.show_grid()

        plotter.add_ground(FL_cp=self.FL_double_wishbone.contact_patch, RL_cp=self.RL_double_wishbone.contact_patch, tire=self.FL_double_wishbone.tire)
        for element in self.elements:
            element.plot_elements(plotter=plotter, verbose=self.verbose)

    def add_slider(self, func: Callable, title: str, bounds: Tuple[float, float], pos: Tuple[Tuple[float, float], Tuple[float, float]]) -> None:
        """
        ## Add Slider

        Adds slider to Plotter window

        Parameters
        ----------
        func : Callable
            Function to execute slider values on
        title : str
            Title of slider
        bounds : Tuple[float, float]
            Limits of slider value
        pos : Tuple[Tuple[float, float], Tuple[float, float]]
            Position of slider (origin at lower left corner of window)
            - Start coordinate [Ratio X, Ratio Y]
            - End Coordinate [Ratio X, Ratio Y]
            - Max coordinate: [1, 1]
            - Min coordinate: [0, 0]
        
        Returns
        ----------
        None
        """
        self.plotter.add_slider(func, title, bounds, pos)

    def steer_slider(self, steer: float) -> None:
        """
        ## Steer Slider

        Steer slider function call

        Parameters
        ----------
        steer : float
            Rack displacement
            - Must use same units as suspension coordinates above

        Returns
        ----------
        None
        """
        self.steer(rack_displacement=steer)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)
    
    def heave_slider(self, heave: float) -> None:
        """
        ## Heave Slider

        Heave slider function call

        Parameters
        ----------
        heave : float
            Vertical translation of sprung mass
            - Must use same units as suspension coordinates above
            - Down is positive

        Returns
        ----------
        None
        """
        self.heave(heave=heave)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def pitch_slider(self, pitch: float) -> None:
        """
        ## Pitch Slider

        Pitch slider function call

        Parameters
        ----------
        pitch : float
            Vehicle pitch in degrees

        Returns
        ----------
        None
        """
        self.pitch(pitch=pitch)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)

    def roll_slider(self, roll: float) -> None:
        """
        ## Roll Slider

        Roll slider function call

        Parameters
        ----------
        roll : float
            Vehicle roll in degrees

        Returns
        ----------
        None
        """
        self.roll(roll=roll)
        self.plot_elements(plotter=self.plotter, verbose=self.verbose)