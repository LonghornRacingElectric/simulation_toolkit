from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from vehicle_model.suspension_model.suspension_elements.quaternary_elements.axle import Axle
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from vehicle_model.suspension_model.assets.interp import interp3d
from vehicle_model._assets.pickle_helpers import pickle_import
from typing import Callable, Sequence, Tuple
import pyvista as pv
import numpy as np
import pickle
import os


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
    FL_bellcrank_params : Sequence[Sequence[float]]
        Relevant bellcrank to shock parameters for front-left suspension assembly
        - Order: [[Pivot], [Pivot Direction], [Shock Outboard], [Shock Inboard]]
    FL_upper : bool
        True if push/pull rod mounts to upper wishbone
    FL_contact_patch : Sequence[float]
        Coordinates of the front-left contact patch
    FL_inclination_angle : float
        Inclination angle of the front-left tire in degrees
    FL_toe : float
        Static toe angle of the front-left tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    FL_rate : float
        Spring rate of the front left assembly in N/m
    FL_weight : float
        Weight of the front left vehicle corner in N

    ---

    FR_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the front-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FR_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the front-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    FR_bellcrank_params : Sequence[Sequence[float]]
        Relevant bellcrank to shock parameters for front-right suspension assembly
        - Order: [[Pivot], [Pivot Direction], [Shock Outboard], [Shock Inboard]]
    FR_upper : bool
        True if push/pull rod mounts to upper wishbone
    FR_contact_patch : Sequence[float]
        Coordinates of the front-right contact patch
    FR_inclination_angle : float
        Inclination angle of the front-right tire in degrees
    FR_toe : float
        Static toe angle of the front-right tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    FR_rate : float
        Spring rate of the front right assembly in N/m
    FR_weight : float
        Weight of the front right vehicle corner in N
    
    ---

    RL_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the rear-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RL_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the rear-left suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RL_bellcrank_params : Sequence[Sequence[float]]
        Relevant bellcrank to shock parameters for rear-left suspension assembly
        - Order: [[Pivot], [Pivot Direction], [Shock Outboard], [Shock Inboard]]
    RL_upper : bool
        True if push/pull rod mounts to upper wishbone
    RL_contact_patch : Sequence[float]
        Coordinates of the rear-left contact patch
    RL_inclination_angle : float
        Inclination angle of the rear-left tire in degrees
    RL_toe : float
        Static toe angle of the rear-left tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    RL_rate : float
        Spring rate of the rear left assembly in N/m
    RL_weight : float
        Weight of the rear left vehicle corner in N
    
    --- 

    RR_inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the rear-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RR_outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the rear-right suspension assembly
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod]]
    RR_bellcrank_params : Sequence[Sequence[float]]
        Relevant bellcrank to shock parameters for rear-right suspension assembly
        - Order: [[Pivot], [Pivot Direction], [Shock Outboard], [Shock Inboard]]
    RR_upper : bool
        True if push/pull rod mounts to upper wishbone
    RR_contact_patch : Sequence[float]
        Coordinates of the rear-right contact patch
    RR_inclination_angle : float
        Inclination angle of the rear-right tire in degrees
    RR_toe : float
        Static toe angle of the rear-right tire in degrees
        - Uses same sign convention as slip angle, NOT symmetric
    RR_rate : float
        Spring rate of the rear right assembly in N/m
    RR_weight : float
        Weight of the rear right vehicle corner in N
    
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
            FL_bellcrank_params: Sequence[Sequence[float]],
            FL_upper: bool,
            FL_contact_patch: Sequence[float],
            FL_inclination_angle: float,
            FL_toe: float,
            FL_rate: float,
            FL_weight: float,

            FR_inboard_points: Sequence[Sequence[float]],
            FR_outboard_points: Sequence[Sequence[float]],
            FR_bellcrank_params: Sequence[Sequence[float]],
            FR_upper: bool,
            FR_contact_patch: Sequence[float],
            FR_inclination_angle: float,
            FR_toe: float,
            FR_rate: float,
            FR_weight: float,

            Fr_ARBK: float,
            Fr_ARBMR: float,

            RL_inboard_points: Sequence[Sequence[float]],
            RL_outboard_points: Sequence[Sequence[float]],
            RL_bellcrank_params: Sequence[Sequence[float]],
            RL_upper: bool,
            RL_contact_patch: Sequence[float],
            RL_inclination_angle: float,
            RL_toe: float,
            RL_rate: float,
            RL_weight: float,

            RR_inboard_points: Sequence[Sequence[float]],
            RR_outboard_points: Sequence[Sequence[float]],
            RR_bellcrank_params: Sequence[Sequence[float]],
            RR_upper: bool,
            RR_contact_patch: Sequence[float],
            RR_inclination_angle: float,
            RR_toe: float,
            RR_rate: float,
            RR_weight: float,

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

        # Initialize state
        self.current_heave = 0
        self.current_pitch = 0
        self.current_roll = 0

        # Initialize each corner assembly
        self.FL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FL_inboard_points,
                                                                outboard_points=FL_outboard_points,
                                                                bellcrank_params=FL_bellcrank_params,
                                                                spring_rate=FL_rate,
                                                                weight=FL_weight,
                                                                upper=FL_upper,
                                                                contact_patch=FL_contact_patch,
                                                                inclination_angle=FL_inclination_angle * np.pi / 180,
                                                                toe=FL_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.FR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FR_inboard_points,
                                                                outboard_points=FR_outboard_points,
                                                                bellcrank_params=FR_bellcrank_params,
                                                                spring_rate=FR_rate,
                                                                weight=FR_weight,
                                                                upper=FR_upper,
                                                                contact_patch=FR_contact_patch,
                                                                inclination_angle=FR_inclination_angle * np.pi / 180,
                                                                toe=FR_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RL_inboard_points,
                                                                outboard_points=RL_outboard_points,
                                                                bellcrank_params=RL_bellcrank_params,
                                                                spring_rate=RL_rate,
                                                                weight=RL_weight,
                                                                upper=RL_upper,
                                                                contact_patch=RL_contact_patch,
                                                                inclination_angle=RL_inclination_angle * np.pi / 180,
                                                                toe=RL_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)
            
        self.RR_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=RR_inboard_points,
                                                                outboard_points=RR_outboard_points,
                                                                bellcrank_params=RR_bellcrank_params,
                                                                spring_rate=RR_rate,
                                                                weight=RR_weight,
                                                                upper=RR_upper,
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

        # Cache kinematics for simulation use
        if os.listdir("./outputs/kin_interp"):
            print("Kin lookup object found\n")
            file_path = "./outputs/kin_interp/" + os.listdir("./outputs/kin_interp")[0]
            with open(file_path, 'rb') as inp:
                lookup_objs = pickle.load(inp)

            self.FL_gamma_lookup = lookup_objs[0]
            self.FL_toe_lookup = lookup_objs[1]
            self.FL_caster_lookup = lookup_objs[2]

            self.FR_gamma_lookup = lookup_objs[3]
            self.FR_toe_lookup = lookup_objs[4]
            self.FR_caster_lookup = lookup_objs[5]
            
            self.RL_gamma_lookup = lookup_objs[6]
            self.RL_toe_lookup = lookup_objs[7]
            self.RL_caster_lookup = lookup_objs[8]

            self.RR_gamma_lookup = lookup_objs[9]
            self.RR_toe_lookup = lookup_objs[10]
            self.RR_caster_lookup = lookup_objs[11]

            self.Fr_Kr_lookup = lookup_objs[12]
            self.Fr_RC_lookup = lookup_objs[13]
            self.Rr_Kr_lookup = lookup_objs[14]
            self.Rr_RC_lookup = lookup_objs[15]
            self.Kp_lookup = lookup_objs[16]
            self.PC_lookup = lookup_objs[17]
        
        else: 
            print("Kin lookup object NOT found | Generating now:\n")
            corners = [self.Fr_axle.left, self.Fr_axle.right, self.Rr_axle.left, self.Rr_axle.right]
            refinement = 10
            steer_sweep = np.linspace(-0.0254 * 2, 0.0254 * 2, refinement)
            heave_sweep = np.linspace(-0.0254 * 5, 0.0254 * 5, refinement)
            pitch_sweep = np.linspace(-5, 5, refinement)
            roll_sweep = np.linspace(-5, 5, refinement)
            
            gamma_angles = {"FL": {"data": [], "lookup": None}, 
                            "FR": {"data": [], "lookup": None}, 
                            "RL": {"data": [], "lookup": None}, 
                            "RR": {"data": [], "lookup": None}}
            toe_angles = {"FL": {"data": [], "lookup": None},
                        "FR": {"data": [], "lookup": None},
                        "RL": {"data": [], "lookup": None},
                        "RR": {"data": [], "lookup": None}}
            caster_angles = {"FL": {"data": [], "lookup": None},
                            "FR": {"data": [], "lookup": None},
                            "RL": {"data": [], "lookup": None},
                            "RR": {"data": [], "lookup": None}}
            FVIC_pos = {"FL": {"data": [], "lookup": None},
                        "FR": {"data": [], "lookup": None},
                        "RL": {"data": [], "lookup": None},
                        "RR": {"data": [], "lookup": None}}
            SVIC_pos = {"FL": {"data": [], "lookup": None},
                        "FR": {"data": [], "lookup": None},
                        "RL": {"data": [], "lookup": None},
                        "RR": {"data": [], "lookup": None}}
            
            roll_stiffness = {"Fr": [], "Rr": []}
            roll_center = {"Fr": [], "Rr": []}
            pitch_stiffness = {"tot": []}
            pitch_center = {"tot": []}
            
            labels = ["FL", "FR", "RL", "RR"]
            
            total_iter = refinement**4
            counter = 0
            for steer in steer_sweep:
                try:
                    self.steer(rack_displacement=steer)
                except:
                    continue
                for heave in heave_sweep:
                    try:
                        self.heave(heave=heave)
                    except:
                        self.heave(heave=heave-0.254/15)
                    for pitch in pitch_sweep:
                        try:
                            self.pitch(pitch=pitch)
                        except:
                            self.pitch(pitch=pitch-10/15)
                        for roll in roll_sweep:
                            print(f"Creating Kin Lookup: {round(counter / total_iter * 100, 4)}%", end="\r")

                            try:
                                self.roll(roll=roll)
                            except:
                                self.roll(roll=roll-10/15)

                            for i in range(len(corners)):
                                gamma_angles[labels[i]]["data"].append(corners[i].inclination_angle)
                                toe_angles[labels[i]]["data"].append(corners[i].toe)
                                caster_angles[labels[i]]["data"].append(corners[i].caster)
                            
                            roll_stiffness["Fr"].append(self.full_suspension.Fr_axle.roll_stiffness)
                            roll_center["Fr"].append(self.full_suspension.Fr_axle.kin_RC.true_KinRC.position)
                            roll_stiffness["Rr"].append(self.full_suspension.Rr_axle.roll_stiffness)
                            roll_center["Rr"].append(self.full_suspension.Rr_axle.kin_RC.true_KinRC.position)
                            
                            pitch_stiffness["tot"].append(self.full_suspension.pitch_stiffness)
                            pitch_center["tot"].append(self.full_suspension.right_kin_PC.true_KinPC.position)

                            counter += 1
            
            lookup_objs = []
            for label in labels:
                lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=gamma_angles[label]))
                lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=toe_angles[label]))
                lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=caster_angles[label]))
                lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=FVIC_pos[label]))
                lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=SVIC_pos[label]))
            
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=roll_stiffness["Fr"]))
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=roll_center["Fr"]))
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=roll_stiffness["Rr"]))
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=roll_center["Rr"]))
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=pitch_stiffness["tot"]))
            lookup_objs.append(interp3d(x=steer_sweep, y=heave_sweep, z=pitch_sweep, w=roll_sweep, v=pitch_center["tot"]))

            with open(f"./outputs/kin_interp/kin_lookup.pkl", 'wb') as outp:
                pickle.dump(lookup_objs, outp, pickle.HIGHEST_PROTOCOL)
            
            self.FL_gamma_lookup = lookup_objs[0]
            self.FL_toe_lookup = lookup_objs[1]
            self.FL_caster_lookup = lookup_objs[2]

            self.FR_gamma_lookup = lookup_objs[3]
            self.FR_toe_lookup = lookup_objs[4]
            self.FR_caster_lookup = lookup_objs[5]
            
            self.RL_gamma_lookup = lookup_objs[6]
            self.RL_toe_lookup = lookup_objs[7]
            self.RL_caster_lookup = lookup_objs[8]

            self.RR_gamma_lookup = lookup_objs[9]
            self.RR_toe_lookup = lookup_objs[10]
            self.RR_caster_lookup = lookup_objs[11]

            self.Fr_Kr_lookup = lookup_objs[12]
            self.Fr_RC_lookup = lookup_objs[13]
            self.Rr_Kr_lookup = lookup_objs[14]
            self.Rr_RC_lookup = lookup_objs[15]
            self.Kp_lookup = lookup_objs[16]
            self.PC_lookup = lookup_objs[17]

        # Elements for plotting
        self.elements = [self.full_suspension]
    
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
        self.current_heave = heave
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
        self.current_pitch = pitch
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
        self.current_roll = roll
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