from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_elements.quinary_elements.full_suspension import FullSuspension
from vehicle_model.suspension_model.suspension_elements.quaternary_elements.axle import Axle
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from vehicle_model.suspension_model.assets.interp import interp3d
from vehicle_model.suspension_model.assets.interp import interp2d
from vehicle_model._assets.pickle_helpers import pickle_import
from typing import Callable, Sequence, Tuple
from matplotlib.figure import Figure
from collections import OrderedDict
from dash import Dash, dash_table
import matplotlib.pyplot as plt
import pyvista as pv
import pandas as pd
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

        # CG location
        self.cg: CG = CG(position=cg_location)

        # Initialize each corner assembly
        self.FL_double_wishbone: DoubleWishbone = DoubleWishbone(inboard_points=FL_inboard_points,
                                                                outboard_points=FL_outboard_points,
                                                                bellcrank_params=FL_bellcrank_params,
                                                                spring_rate=FL_rate,
                                                                weight=FL_weight,
                                                                cg=self.cg,
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
                                                                cg=self.cg,
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
                                                                cg=self.cg,
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
                                                                cg=self.cg,
                                                                upper=RR_upper,
                                                                contact_patch=RR_contact_patch,
                                                                inclination_angle=RR_inclination_angle * np.pi / 180,
                                                                toe=RR_toe * np.pi / 180,
                                                                tire_radius=tire_radius,
                                                                tire_width=tire_width,
                                                                show_ICs=show_ICs)

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
            self.FL_FVIC_lookup = lookup_objs[3]
            self.FL_SVIC_lookup = lookup_objs[4]

            self.FR_gamma_lookup = lookup_objs[5]
            self.FR_toe_lookup = lookup_objs[6]
            self.FR_caster_lookup = lookup_objs[7]
            self.FR_FVIC_lookup = lookup_objs[8]
            self.FR_SVIC_lookup = lookup_objs[9]
            
            self.RL_gamma_lookup = lookup_objs[10]
            self.RL_toe_lookup = lookup_objs[11]
            self.RL_caster_lookup = lookup_objs[12]
            self.RL_FVIC_lookup = lookup_objs[13]
            self.RL_SVIC_lookup = lookup_objs[14]

            self.RR_gamma_lookup = lookup_objs[15]
            self.RR_toe_lookup = lookup_objs[16]
            self.RR_caster_lookup = lookup_objs[17]
            self.RR_FVIC_lookup = lookup_objs[18]
            self.RR_SVIC_lookup = lookup_objs[19]

            self.Fr_Kr_lookup = lookup_objs[20]
            self.Fr_RC_lookup = lookup_objs[21]
            self.Rr_Kr_lookup = lookup_objs[22]
            self.Rr_RC_lookup = lookup_objs[23]
            self.Kp_lookup = lookup_objs[24]
            self.PC_lookup = lookup_objs[25]
        
        else: 
            print("Kin lookup object NOT found | Generating now:\n")
            corners = [self.Fr_axle.left, self.Fr_axle.right, self.Rr_axle.left, self.Rr_axle.right]
            refinement = 20
            steer_sweep = np.linspace(-0.0254 * 3, 0.0254 * 3, refinement)
            jounce_sweep = np.linspace(-0.0254 * 5, 0.0254 * 5, refinement)
            
            gamma_angles = {"FL": [], "FR": [], "RL": [], "RR": []}
            toe_angles = {"FL": [], "FR": [], "RL": [], "RR": []}
            caster_angles = {"FL": [], "FR": [], "RL": [], "RR": []}
            FVIC_pos = {"FL": [], "FR": [], "RL": [], "RR": []}
            SVIC_pos = {"FL": [], "FR": [], "RL": [], "RR": []}
            
            roll_stiffness = {"Fr": [], "Rr": []}
            roll_center = {"Fr": [], "Rr": []}
            pitch_stiffness = {"tot": []}
            pitch_center = {"tot": []}
            
            labels = ["FL", "FR", "RL", "RR"]
            
            total_iter = refinement**2
            counter = 0
            for steer in steer_sweep:
                self.full_suspension.Fr_axle.left.steer(steer=steer)
                self.full_suspension.Fr_axle.right.steer(steer=steer)
                self.full_suspension.Rr_axle.left.steer(steer=steer)
                self.full_suspension.Rr_axle.right.steer(steer=steer)
                for jounce in jounce_sweep:
                    print(f"Creating Kin Lookup: {round(counter / total_iter * 100, 4)}%", end="\r")

                    self.full_suspension.Fr_axle.left.jounce(jounce=jounce)
                    self.full_suspension.Fr_axle.right.jounce(jounce=jounce)
                    self.full_suspension.Rr_axle.left.jounce(jounce=jounce)
                    self.full_suspension.Rr_axle.right.jounce(jounce=jounce)

                    for i in range(len(corners)):
                        gamma_angles[labels[i]].append(corners[i].inclination_angle)
                        toe_angles[labels[i]].append(corners[i].toe)
                        caster_angles[labels[i]].append(corners[i].caster)
                    
                    roll_stiffness["Fr"].append(self.full_suspension.Fr_axle.roll_stiffness)
                    roll_center["Fr"].append(self.full_suspension.Fr_axle.kin_RC.true_KinRC.position)
                    roll_stiffness["Rr"].append(self.full_suspension.Rr_axle.roll_stiffness)
                    roll_center["Rr"].append(self.full_suspension.Rr_axle.kin_RC.true_KinRC.position)
                    
                    pitch_stiffness["tot"].append(self.full_suspension.pitch_stiffness)
                    pitch_center["tot"].append(self.full_suspension.right_kin_PC.true_KinPC.position)

                    counter += 1

            lookup_objs = []
            for label in labels:
                lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=gamma_angles[label]))
                lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=toe_angles[label]))
                lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=caster_angles[label]))
                lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=FVIC_pos[label]))
                lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=SVIC_pos[label]))
            
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=roll_stiffness["Fr"]))
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=roll_center["Fr"]))
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=roll_stiffness["Rr"]))
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=roll_center["Rr"]))
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=pitch_stiffness["tot"]))
            lookup_objs.append(interp2d(x=steer_sweep, y=jounce_sweep, z=pitch_center["tot"]))

            with open(f"./outputs/kin_interp/kin_lookup.pkl", 'wb') as outp:
                pickle.dump(lookup_objs, outp, pickle.HIGHEST_PROTOCOL)
            
            self.FL_gamma_lookup = lookup_objs[0]
            self.FL_toe_lookup = lookup_objs[1]
            self.FL_caster_lookup = lookup_objs[2]
            self.FL_FVIC_lookup = lookup_objs[3]
            self.FL_SVIC_lookup = lookup_objs[4]

            self.FR_gamma_lookup = lookup_objs[5]
            self.FR_toe_lookup = lookup_objs[6]
            self.FR_caster_lookup = lookup_objs[7]
            self.FR_FVIC_lookup = lookup_objs[8]
            self.FR_SVIC_lookup = lookup_objs[9]
            
            self.RL_gamma_lookup = lookup_objs[10]
            self.RL_toe_lookup = lookup_objs[11]
            self.RL_caster_lookup = lookup_objs[12]
            self.RL_FVIC_lookup = lookup_objs[13]
            self.RL_SVIC_lookup = lookup_objs[14]

            self.RR_gamma_lookup = lookup_objs[15]
            self.RR_toe_lookup = lookup_objs[16]
            self.RR_caster_lookup = lookup_objs[17]
            self.RR_FVIC_lookup = lookup_objs[18]
            self.RR_SVIC_lookup = lookup_objs[19]

            self.Fr_Kr_lookup = lookup_objs[20]
            self.Fr_RC_lookup = lookup_objs[21]
            self.Rr_Kr_lookup = lookup_objs[22]
            self.Rr_RC_lookup = lookup_objs[23]
            self.Kp_lookup = lookup_objs[24]
            self.PC_lookup = lookup_objs[25]

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
    
    def generate_report(self) -> None:
        data = OrderedDict(
            [
                ("Dimensions", ["Overall Dimensions", 
                                "Wheelbase and Track", 
                                "Center of Gravity Design Height", 
                                "Mass without Driver", 
                                "Weight Distribution with 68kg Driver"]),
                ("Units", ["mm",
                           "mm",
                           "mm",
                           "kg",
                           "-"]),
                ("  ", ["Length", 
                      "Wheelbase", 
                      "CG Height", 
                      "Front", 
                      "% Front"]),
                (" ", [" ",
                      " ",
                      " ",
                      " ",
                      " "]),
                ("", ["Width",
                      "Front Track",
                      "Confirmed Via",
                      "Rear",
                      "% Left"])
            ]
        )

        df = pd.DataFrame(data)

        app = Dash(__name__)

        app.layout = dash_table.DataTable(
            data=df.to_dict('records'),
            columns=[{'id': c, 'name': c} for c in df.columns],
            style_header={'border': '1px solid black', 'backgroundColor': '#84a0a8'},
            style_cell={'textAlign': 'center', 'border': '1px solid grey'},
            merge_duplicate_headers=True
        )

        app.run(debug=False)

    def generate_kin_plots(self, steer_sweep: np.ndarray, heave_sweep: np.ndarray, pitch_sweep: np.ndarray, roll_sweep: np.ndarray) -> None:
        self.steer_sweep = steer_sweep
        self.heave_sweep = heave_sweep
        self.pitch_sweep = pitch_sweep
        self.roll_sweep = roll_sweep

        # Store double wishbones
        self.FL_dw = self.full_suspension.Fr_axle.left
        self.FR_dw = self.full_suspension.Fr_axle.right
        self.RL_dw = self.full_suspension.Rr_axle.left
        self.RR_dw = self.full_suspension.Rr_axle.right

        # Store geometric properties
        self.FL_cg_x = abs(self.FL_dw.contact_patch.position[0] - self.cg.position[0])
        self.FR_cg_x = abs(self.FR_dw.contact_patch.position[0] - self.cg.position[0])
        self.RL_cg_x = abs(self.RL_dw.contact_patch.position[0] - self.cg.position[0])
        self.RR_cg_x = abs(self.RR_dw.contact_patch.position[0] - self.cg.position[0])
        self.FL_cg_y = abs(self.FL_dw.contact_patch.position[1] - self.cg.position[1])
        self.FR_cg_y = abs(self.FR_dw.contact_patch.position[1] - self.cg.position[1])
        self.RL_cg_y = abs(self.RL_dw.contact_patch.position[1] - self.cg.position[1])
        self.RR_cg_y = abs(self.RR_dw.contact_patch.position[1] - self.cg.position[1])

        self.FL_bump_gamma = []
        self.FR_bump_gamma = []
        self.RL_bump_gamma = []
        self.RR_bump_gamma = []

        self.FL_bump_toe = []
        self.FR_bump_toe = []
        self.RL_bump_toe = []
        self.RR_bump_toe = []

        self.FL_roll_gamma = []
        self.FR_roll_gamma = []
        self.RL_roll_gamma = []
        self.RR_roll_gamma = []

        self.FL_2D_bump_jounce = []
        self.FR_2D_bump_jounce = []
        self.RL_2D_bump_jounce = []
        self.RR_2D_bump_jounce = []
        self.FL_2D_bump_steer = []
        self.FR_2D_bump_steer = []
        self.RL_2D_bump_steer = []
        self.RR_2D_bump_steer = []
        self.FL_2D_bump_gamma = []
        self.FR_2D_bump_gamma = []
        self.RL_2D_bump_gamma = []
        self.RR_2D_bump_gamma = []

        self.full_suspension.reset_position()

        counter = 0
        total_count_1D = len(self.steer_sweep)
        total_count_2D = len(self.steer_sweep)**2

        for heave in self.heave_sweep:
            counter += 1
            # self.generate_kin_helper(steer=0, heave=heave, pitch=0, roll=0)
            self.FL_dw.jounce(jounce=heave)
            self.FR_dw.jounce(jounce=heave)
            self.RL_dw.jounce(jounce=heave)
            self.RR_dw.jounce(jounce=heave)
            self.FL_bump_gamma.append(self.FL_dw.inclination_angle * 180 / np.pi)
            self.FR_bump_gamma.append(self.FR_dw.inclination_angle * 180 / np.pi)
            self.RL_bump_gamma.append(self.RL_dw.inclination_angle * 180 / np.pi)
            self.RR_bump_gamma.append(self.RR_dw.inclination_angle * 180 / np.pi)

            self.FL_bump_toe.append(self.FL_dw.toe * 180 / np.pi)
            self.FR_bump_toe.append(self.FR_dw.toe * 180 / np.pi)
            self.RL_bump_toe.append(self.RL_dw.toe * 180 / np.pi)
            self.RR_bump_toe.append(self.RR_dw.toe * 180 / np.pi)
            print(f"Bump Toe/Camber Progress: {round(counter / total_count_1D * 100, 1)}%", end="\r")

        print()
        counter = 0
        for steer in self.steer_sweep:
            for heave in self.heave_sweep:
                counter += 1
                self.generate_kin_helper(steer=steer, heave=heave, pitch=0, roll=0)
                self.FL_2D_bump_jounce.append(heave * 1000)
                self.FR_2D_bump_jounce.append(heave * 1000)
                self.RL_2D_bump_jounce.append(heave * 1000)
                self.RR_2D_bump_jounce.append(heave * 1000)

                self.FL_2D_bump_steer.append(steer * 1000)
                self.FR_2D_bump_steer.append(steer * 1000)
                self.RL_2D_bump_steer.append(steer * 1000)
                self.RR_2D_bump_steer.append(steer * 1000)

                self.FL_2D_bump_gamma.append(self.FL_dw.inclination_angle * 180 / np.pi)
                self.FR_2D_bump_gamma.append(self.FR_dw.inclination_angle * 180 / np.pi)
                self.RL_2D_bump_gamma.append(self.RL_dw.inclination_angle * 180 / np.pi)
                self.RR_2D_bump_gamma.append(self.RR_dw.inclination_angle * 180 / np.pi)
                print(f"Suspension Map Progress: {round(counter / total_count_2D * 100, 1)}%", end="\r")

        print()
        counter = 0
        for roll in self.roll_sweep:
            counter += 1
            self.generate_kin_helper(steer=0, heave=0, pitch=0, roll=roll)
            self.FL_roll_gamma.append(self.FL_dw.inclination_angle * 180 / np.pi + roll)
            self.FR_roll_gamma.append(self.FR_dw.inclination_angle * 180 / np.pi + roll)
            self.RL_roll_gamma.append(self.RL_dw.inclination_angle * 180 / np.pi + roll)
            self.RR_roll_gamma.append(self.RR_dw.inclination_angle * 180 / np.pi + roll)
            print(f"Roll Camber Progress: {round(counter / total_count_1D * 100, 1)}%", end="\r")

        self.plot()
    
    def generate_kin_helper(self, steer: float, heave: float, pitch: float, roll: float):
        FL_jounce = heave + (self.FL_cg_x * np.sin(pitch * np.pi / 180) + self.FL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
        FR_jounce = heave + (self.FR_cg_x * np.sin(pitch * np.pi / 180) + self.FR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
        RL_jounce = heave + (self.RL_cg_x * np.sin(pitch * np.pi / 180) + self.RL_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))
        RR_jounce = heave + (self.RR_cg_x * np.sin(pitch * np.pi / 180) + self.RR_cg_y * np.cos(pitch * np.pi / 180) * np.sin(roll * np.pi / 180)) / (np.cos(pitch * np.pi / 180) * np.cos(roll * np.pi / 180))

        self.FL_dw.steer(steer=steer)
        self.FR_dw.steer(steer=steer)

        self.FL_dw.jounce(jounce=FL_jounce)
        self.FR_dw.jounce(jounce=FR_jounce)
        self.RL_dw.jounce(jounce=RL_jounce)
        self.RR_dw.jounce(jounce=RR_jounce)

    def plot(self) -> Figure:
        # Create 1D Figure
        fig, ax = plt.subplots(nrows=1, ncols=4)

        ax[0].plot(self.heave_sweep * 1000, self.FL_bump_gamma)
        ax[0].set_xlabel("Jounce (mm)")
        ax[0].set_ylabel("Inclination Angle (deg)")

        ax[1].plot(self.roll_sweep, self.FL_roll_gamma)
        ax[1].set_xlabel("Roll (deg)")
        ax[1].set_ylabel("Inclination Angle (deg)")

        ax[2].plot(self.heave_sweep * 1000, self.FL_bump_toe)
        ax[2].set_xlabel("FL Jounce (mm)")
        ax[2].set_ylabel("FL Toe Angle (deg)")

        ax[3].plot(self.heave_sweep * 1000, self.RL_bump_toe)
        ax[3].set_xlabel("RL Jounce (mm)")
        ax[3].set_ylabel("RL Toe Angle (deg)")
        # ax[2].plot()
        plt.show()

        # Create 2D figure
        fig = plt.figure(figsize=[11, 8.5])
        fig.suptitle("Tire Fit Plots", fontsize=20)
        
        ### Create subplots ###

        # Pure slip Fx
        ax = fig.add_subplot(1, 1, 1, projection='3d')
        
        steer_sweep, heave_sweep = np.meshgrid(self.steer_sweep * 1000, self.heave_sweep * 1000)
        gamma = np.array(self.FL_2D_bump_gamma).reshape((len(self.steer_sweep * 1000), len(self.heave_sweep * 1000)))
        ax.plot_surface(steer_sweep, heave_sweep, gamma)
        ax.set_xlabel("Rack Displacement (mm)")
        ax.set_ylabel("Jounce (mm)")
        ax.set_zlabel("Inclination Angle (deg)")

        # self.format_fig(ax=ax,
        #                 title=f"Fx(Fz, alpha=0, kappa, gamma={round(gamma, 3)})",
        #                 xlabel="Fz (N)",
        #                 ylabel="Kappa (-)",
        #                 zlabel="Fx (N)",
        #                 title_size=8,
        #                 label_size=6)

        plt.show()