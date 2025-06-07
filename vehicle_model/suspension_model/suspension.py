from vehicle_model.suspension_model.suspension_elements._5_elements.quarter_car import QuarterCar
from vehicle_model.suspension_model.suspension_elements._2_elements.stabar import Stabar
from vehicle_model.suspension_model.suspension_elements._2_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from vehicle_model.suspension_model.suspension_data import SuspensionData
from _4_custom_libraries.misc_math import rotation_matrix

from typing import Tuple, Union, cast
from dataclasses import dataclass
from copy import deepcopy
import numpy as np


@dataclass
class Suspension:
    """
    ## Suspension Model

    Designed to model kinematics and force-based properties

    ###### Note, all conventions comply with SAE-J670 Z-up
    ###### Standard order of displacement: heave (T) -> yaw (R) -> pitch (R) -> roll (R)

    Parameters
    ----------
    sus_data : SuspensionData
        Suspension parameter definition
    """
    sus_data: SuspensionData

    def __post_init__(self) -> None:
        self.FL_quarter_car = self.sus_data.FL_quarter_car
        self.FR_quarter_car = self.sus_data.FR_quarter_car
        self.RL_quarter_car = self.sus_data.RL_quarter_car
        self.RR_quarter_car = self.sus_data.RR_quarter_car

        self.state_cache: dict[str, "Suspension"]

        self.state: dict[str, float] = {"veh_CG_x": self.veh_CG[0],
                                        "veh_CG_y": self.veh_CG[1],
                                        "veh_CG_z": self.veh_CG[2],
                                        "FL_gamma": self.FL_gamma,
                                        "FR_gamma": self.FR_gamma,
                                        "RL_gamma": self.RL_gamma,
                                        "RR_gamma": self.RR_gamma,
                                        "FL_delta": self.FL_delta,
                                        "FR_delta": self.FR_delta,
                                        "RL_delta": self.RL_delta,
                                        "RR_delta": self.RR_delta,
                                        "FL_caster": self.FL_caster,
                                        "FR_caster": self.FR_caster,
                                        "RL_caster": self.RL_caster,
                                        "RR_caster": self.RR_caster,
                                        "FL_kpi": self.FL_kpi,
                                        "FR_kpi": self.FR_kpi,
                                        "RL_kpi": self.RL_kpi,
                                        "RR_kpi": self.RR_kpi,
                                        "FL_mech_trail": self.FL_mech_trail,
                                        "FR_mech_trail": self.FR_mech_trail,
                                        "RL_mech_trail": self.RL_mech_trail,
                                        "RR_mech_trail": self.RR_mech_trail,
                                        "FL_scrub": self.FL_scrub,
                                        "FR_scrub": self.FR_scrub,
                                        "RL_scrub": self.RL_scrub,
                                        "RR_scrub": self.RR_scrub,
                                        "FL_FVIC_y": self.FL_FVIC[1],
                                        "FR_FVIC_y": self.FR_FVIC[1],
                                        "RL_FVIC_y": self.RL_FVIC[1],
                                        "RR_FVIC_y": self.RR_FVIC[1],
                                        "FL_FVIC_z": self.FL_FVIC[2],
                                        "FR_FVIC_z": self.FR_FVIC[2],
                                        "RL_FVIC_z": self.RL_FVIC[2],
                                        "RR_FVIC_z": self.RR_FVIC[2],
                                        "FL_SVIC_x": self.FL_SVIC[0],
                                        "FR_SVIC_x": self.FR_SVIC[0],
                                        "RL_SVIC_x": self.RL_SVIC[0],
                                        "RR_SVIC_x": self.RR_SVIC[0],
                                        "FL_SVIC_z": self.FL_SVIC[2],
                                        "FR_SVIC_z": self.FR_SVIC[2],
                                        "RL_SVIC_z": self.RL_SVIC[2],
                                        "RR_SVIC_z": self.RR_SVIC[2],
                                        "Fr_RC_y": self.Fr_RC[1],
                                        "Fr_RC_z": self.Fr_RC[2],
                                        "Rr_RC_y": self.Rr_RC[1],
                                        "Rr_RC_z": self.Rr_RC[2],
                                        "FL_spring_MR": self.FL_spring_MR,
                                        "FR_spring_MR": self.FR_spring_MR,
                                        "RL_spring_MR": self.RL_spring_MR,
                                        "RR_spring_MR": self.RR_spring_MR,
                                        "Fr_stabar_MR_rot": self.Fr_stabar_MR[0],
                                        "Fr_stabar_MR_trans": self.Fr_stabar_MR[1],
                                        "Rr_stabar_MR_rot": self.Rr_stabar_MR[0],
                                        "Rr_stabar_MR_trans": self.Rr_stabar_MR[1],}

        self.FL_nodes = self.sus_data.FL_nodes
        self.FL_links = self.sus_data.FL_links

        self.FR_nodes = self.sus_data.FR_nodes
        self.FR_links = self.sus_data.FR_links

        self.RL_nodes = self.sus_data.RL_nodes
        self.RL_links = self.sus_data.RL_links

        self.RR_nodes = self.sus_data.RR_nodes
        self.RR_links = self.sus_data.RR_links

        self.Fr_stabar_links = self.sus_data.Fr_stabar_links
        self.Rr_stabar_links = self.sus_data.Rr_stabar_links

        self.tires = [self.sus_data.FL_quarter_car.tire,
                      self.sus_data.FR_quarter_car.tire,
                      self.sus_data.RL_quarter_car.tire,
                      self.sus_data.RR_quarter_car.tire]

        self.tire_nodes = [x.contact_patch for x in self.tires]

        self.CG_node = self.sus_data.CG_node
        self.total_mass = self.sus_data.total_mass

    def steer(self, hwa: float) -> None:
        """
        ## Steer

        Steer front axle

        Parameters
        ----------
        hwa : float
            Handwheel angle in degrees

        Returns
        -------
        None
        """
        self.FL_quarter_car.steer(rack_displacement = hwa / 360 * self.sus_data.steering_ratio)
        self.FR_quarter_car.steer(rack_displacement = hwa / 360 * self.sus_data.steering_ratio)
        self._update_state()
    
    def heave(self, heave: Union[None, float]) -> None:
        """
        ## Heave

        Jounces all wheels by the same amount

        Parameters
        ----------
        heave : float
            Vertical travel of contact patch in meters
        
        Returns
        -------
        None
        """
        if heave == None:
            self.FL_quarter_car.jounce(jounce=0)
            self.FR_quarter_car.jounce(jounce=0)
            self.RL_quarter_car.jounce(jounce=0)
            self.RR_quarter_car.jounce(jounce=0)
        else:
            self.FL_quarter_car._jounce_persistent(jounce=heave)
            self.FR_quarter_car._jounce_persistent(jounce=heave)
            self.RL_quarter_car._jounce_persistent(jounce=heave)
            self.RR_quarter_car._jounce_persistent(jounce=heave)
        
        self._update_state()
    
    def pitch(self, pitch: Union[None, float], n_steps: int = 1, update_state: bool = True) -> None:
        """
        ## Pitch

        Pitches vehicle about kinematic pitch center

        Parameters
        ----------
        pitch : float
            Vehicle pitch in degrees
        n_steps : int
            Number of increments to reach desired pitch
        update_state : bool
            Whether to update all variables for the new state
        
        Returns
        -------
        None
        """
        for i in range(n_steps):
            FL_cp = self.FL_quarter_car.tire.contact_patch
            FR_cp = self.FR_quarter_car.tire.contact_patch
            RL_cp = self.RL_quarter_car.tire.contact_patch
            RR_cp = self.RR_quarter_car.tire.contact_patch

            left_pitch = 180 / np.pi * np.arctan(FL_cp[2] - RL_cp[2]) / (FL_cp[0] - RL_cp[0])
            right_pitch = 180 / np.pi * np.arctan(FR_cp[2] - RR_cp[2]) / (FR_cp[0] - RR_cp[0])
            
            FL_cp = self._sprung_to_global(node=FL_cp)
            FR_cp = self._sprung_to_global(node=FR_cp)
            RL_cp = self._sprung_to_global(node=RL_cp)
            RR_cp = self._sprung_to_global(node=RR_cp)
            
            left_PC = self.left_PC
            right_PC = self.right_PC

            FL_jounce = -1 * (left_PC[0] - FL_cp[0]) * np.tan((pitch - left_pitch) / (n_steps - i) * np.pi / 180)
            FR_jounce = -1 * (right_PC[0] - FR_cp[0]) * np.tan((pitch - right_pitch) / (n_steps - i) * np.pi / 180)

            RL_jounce = -1 * (left_PC[0] - RL_cp[0]) * np.tan((pitch - left_pitch) / (n_steps - i) * np.pi / 180)
            RR_jounce = -1 * (right_PC[0] - RR_cp[0]) * np.tan((pitch - right_pitch) / (n_steps - i) * np.pi / 180)

            self.FL_quarter_car._jounce_persistent(jounce=FL_jounce)
            self.FR_quarter_car._jounce_persistent(jounce=FR_jounce)
            self.RL_quarter_car._jounce_persistent(jounce=RL_jounce)
            self.RR_quarter_car._jounce_persistent(jounce=RR_jounce)
        
        if update_state:
            self._update_state()
        
    def roll(self, roll: Union[None, float], n_steps: int = 1, update_state: bool = True) -> None:
        """
        ## Roll

        Rolls vehicle about kinematic roll center

        Parameters
        ----------
        roll : float
            Vehicle roll in degrees
        n_steps : int
            Number of increments to reach desired roll
        update_state : bool
            Whether to update all variables for the new state
        
        Returns
        -------
        None
        """
        for i in range(n_steps):
            FL_cp = self.FL_quarter_car.tire.contact_patch
            FR_cp = self.FR_quarter_car.tire.contact_patch
            RL_cp = self.RL_quarter_car.tire.contact_patch
            RR_cp = self.RR_quarter_car.tire.contact_patch

            Fr_roll = -180 / np.pi * np.arctan(FL_cp[2] - FR_cp[2]) / (FL_cp[1] - FR_cp[1])
            Rr_roll = -180 / np.pi * np.arctan(RL_cp[2] - RR_cp[2]) / (RL_cp[1] - RR_cp[1])

            FL_cp = self._sprung_to_global(node=FL_cp)
            FR_cp = self._sprung_to_global(node=FR_cp)
            RL_cp = self._sprung_to_global(node=RL_cp)
            RR_cp = self._sprung_to_global(node=RR_cp)
            
            Fr_RC = self.Fr_RC
            Rr_RC = self.Rr_RC

            FL_jounce = (Fr_RC[1] - FL_cp[1]) * np.tan((roll - Fr_roll) / (n_steps - i) * np.pi / 180)
            FR_jounce = (Fr_RC[1] - FR_cp[1]) * np.tan((roll - Fr_roll) / (n_steps - i) * np.pi / 180)

            RL_jounce = (Rr_RC[1] - RL_cp[1]) * np.tan((roll - Rr_roll) / (n_steps - i) * np.pi / 180)
            RR_jounce = (Rr_RC[1] - RR_cp[1]) * np.tan((roll - Rr_roll) / (n_steps - i) * np.pi / 180)

            self.FL_quarter_car._jounce_persistent(jounce=FL_jounce)
            self.FR_quarter_car._jounce_persistent(jounce=FR_jounce)
            self.RL_quarter_car._jounce_persistent(jounce=RL_jounce)
            self.RR_quarter_car._jounce_persistent(jounce=RR_jounce)
        
        if update_state:
            self._update_state()

    def FL_jounce(self, jounce: float) -> None:
        """
        ## Front-Left Jounce

        Jounces front-left wheel

        Parameters
        ----------
        jounce : float
            Vertical travel of the front-left contact patch in meters

        Returns
        -------
        None
        """
        self.FL_quarter_car.jounce(jounce=jounce)
        self._update_state()
    
    def FR_jounce(self, jounce: float) -> None:
        """
        ## Front-Right Jounce

        Jounces front-right wheel

        Parameters
        ----------
        jounce : float
            Vertical travel of the front-right contact patch in meters

        Returns
        -------
        None
        """
        self.FR_quarter_car.jounce(jounce=jounce)
        self._update_state()
    
    def RL_jounce(self, jounce: float) -> None:
        """
        ## Rear-Left Jounce

        Jounces rear-left wheel

        Parameters
        ----------
        jounce : float
            Vertical travel of the rear-left contact patch in meters

        Returns
        -------
        None
        """
        self.RL_quarter_car.jounce(jounce=jounce)
        self._update_state()
    
    def RR_jounce(self, jounce: float) -> None:
        """
        ## Rear-Right Jounce

        Jounces rear-right wheel

        Parameters
        ----------
        jounce : float
            Vertical travel of the rear-right contact patch in meters

        Returns
        -------
        None
        """
        self.RR_quarter_car.jounce(jounce=jounce)
        self._update_state()

    def _caster_calculation(self, CP_1: Node, CP_2: Node, CP_3: Node, quarter_car: QuarterCar):
        """
        ## Caster Calculation

        Calculates caster

        Parameters
        ----------
        CP_1 : Node
            One contact patch, other than desired tire

        CP_2 : Node
            One contact patch, other than desired tire and CP_1
        
        CP_3 : Node
            One contact patch, other than desired tire, CP_1, and CP_2

        quarter_car : QuarterCar
            QuarterCar with desired caster

        Returns
        -------
        float
            Caster of given tire in degrees
        """
        # Equation of ground plane where y = contact_patch[0]
        cp_1 = np.array(CP_1.position)
        cp_2 = np.array(CP_2.position)
        cp_3 = np.array(CP_3.position)

        ground_normal = np.cross((cp_2 - cp_1), (cp_3 - cp_1))
        a1 = ground_normal[0]
        c1 = ground_normal[2]

        UCA_outboard = quarter_car.upper_wishbone.fore_link.outboard_node.position
        LCA_outboard = quarter_car.lower_wishbone.fore_link.outboard_node.position

        UCA_outboard_xz = [UCA_outboard[0], UCA_outboard[2]]
        LCA_outboard_xz = [LCA_outboard[0], LCA_outboard[2]]

        a2 = (UCA_outboard_xz[1] - LCA_outboard_xz[1])
        c2 = (LCA_outboard_xz[0] - UCA_outboard_xz[0])

        angle_mag = np.arccos(abs(a1 * a2 + c1 * c2) / (np.sqrt(a1**2 + c1**2) * np.sqrt(a2**2 + c2**2)))
        angle_dir = 1 if LCA_outboard_xz[0] > UCA_outboard_xz[0] else -1

        return (90 - angle_mag * 180 / np.pi) * angle_dir

    def _kpi_calculation(self, CP_1: Node, CP_2: Node, CP_3: Node, quarter_car: QuarterCar):
        """
        ## KPI Calculation

        Calculates kingpin inclination

        Parameters
        ----------
        CP_1 : Node
            One contact patch, other than desired tire

        CP_2 : Node
            One contact patch, other than desired tire and CP_1
        
        CP_3 : Node
            One contact patch, other than desired tire, CP_1, and CP_2

        quarter_car : QuarterCar
            QuarterCar with desired kpi

        Returns
        -------
        float
            Kingpin inclination of given tire in degrees
        """
        # Equation of ground plane where x = contact_patch[0]
        cp_1 = np.array(CP_1.position)
        cp_2 = np.array(CP_2.position)
        cp_3 = np.array(CP_3.position)

        ground_normal = np.cross((cp_2 - cp_1), (cp_3 - cp_1))
        a1 = ground_normal[1]
        c1 = ground_normal[2]

        UCA_outboard = quarter_car.upper_wishbone.fore_link.outboard_node.position
        LCA_outboard = quarter_car.lower_wishbone.fore_link.outboard_node.position

        UCA_outboard_yz = [UCA_outboard[1], UCA_outboard[2]]
        LCA_outboard_yz = [LCA_outboard[1], LCA_outboard[2]]

        a2 = (UCA_outboard_yz[1] - LCA_outboard_yz[1])
        c2 = (LCA_outboard_yz[0] - UCA_outboard_yz[0])

        angle_mag = np.arccos(abs(a1 * a2 + c1 * c2) / (np.sqrt(a1**2 + c1**2) * np.sqrt(a2**2 + c2**2)))
        angle_dir = 1 if LCA_outboard_yz[0] > UCA_outboard_yz[0] else -1

        return (90 - angle_mag * 180 / np.pi) * angle_dir * np.sign(quarter_car.tire.contact_patch[1])

    def _scrub_calculation(self, CP_1: Node, CP_2: Node, CP_3: Node, quarter_car: QuarterCar):
        """
        ## Scrub Calculation

        Calculates scrub radius

        Parameters
        ----------
        CP_1 : Node
            One contact patch, other than desired tire

        CP_2 : Node
            One contact patch, other than desired tire and CP_1
        
        CP_3 : Node
            One contact patch, other than desired tire, CP_1, and CP_2

        quarter_car : QuarterCar
            QuarterCar with desired scrub radius

        Returns
        -------
        float
            Scrub radius of given tire in degrees
        """
        # Define ground plane
        cp_1 = np.array(CP_1.position)
        cp_2 = np.array(CP_2.position)
        cp_3 = np.array(CP_3.position)

        ground_normal = np.cross((cp_2 - cp_1), (cp_3 - cp_1))
        p_0 = cp_1

        # Kingpin
        UCA_outboard = np.array(quarter_car.upper_wishbone.fore_link.outboard_node.position)
        LCA_outboard = np.array(quarter_car.lower_wishbone.fore_link.outboard_node.position)

        v = LCA_outboard - UCA_outboard
        p_1 = UCA_outboard

        # Intersection parameter value
        t = -1 * np.dot((p_1 - p_0), ground_normal) / (np.dot(v, ground_normal))

        intersection_pt = p_1 + np.array(v) * t

        # Calculate scrub magnitude
        cp = quarter_car.tire.contact_patch
        cp_proj = [cp[1], cp[2]]
        inter_proj = [intersection_pt[1], intersection_pt[2]]

        scrub_mag = np.sqrt((cp_proj[0] - inter_proj[0])**2 + (cp_proj[1] - inter_proj[1])**2)
        scrub_dir = 1 if cp[1] > intersection_pt[1] else -1

        return scrub_mag * scrub_dir * np.sign(cp[1])

    def _mech_trail_calculation(self, CP_1: Node, CP_2: Node, CP_3: Node, quarter_car: QuarterCar):
        """
        ## Mech Trail Calculation

        Calculates mechanical trail

        Parameters
        ----------
        CP_1 : Node
            One contact patch, other than desired tire

        CP_2 : Node
            One contact patch, other than desired tire and CP_1
        
        CP_3 : Node
            One contact patch, other than desired tire, CP_1, and CP_2

        quarter_car : QuarterCar
            QuarterCar with desired mechanical trail

        Returns
        -------
        float
            Mechanical trail of given tire in degrees
        """
        # Define ground plane
        cp_1 = np.array(CP_1.position)
        cp_2 = np.array(CP_2.position)
        cp_3 = np.array(CP_3.position)

        ground_normal = np.cross((cp_2 - cp_1), (cp_3 - cp_1))
        p_0 = cp_1

        # Kingpin
        UCA_outboard = np.array(quarter_car.upper_wishbone.fore_link.outboard_node.position)
        LCA_outboard = np.array(quarter_car.lower_wishbone.fore_link.outboard_node.position)

        v = LCA_outboard - UCA_outboard
        p_1 = UCA_outboard

        # Intersection parameter value
        t = -1 * np.dot((p_1 - p_0), ground_normal) / (np.dot(v, ground_normal))

        intersection_pt = p_1 + np.array(v) * t

        # Calculate scrub magnitude
        cp = quarter_car.tire.contact_patch
        cp_proj = [cp[0], cp[2]]
        inter_proj = [intersection_pt[0], intersection_pt[2]]

        mech_trail_mag = np.sqrt((cp_proj[0] - inter_proj[0])**2 + (cp_proj[1] - inter_proj[1])**2)
        mech_trail_dir = 1 if intersection_pt[0] > cp[0] else -1

        return mech_trail_mag * mech_trail_dir

    def _gamma_calculation(self, CP_1: Node, CP_2: Node, CP_3: Node, tire: Tire) -> float:
        """
        ## Inclination Angle Calculation

        Calculates tire inclination angle

        Parameters
        ----------
        CP_1 : Node
            One contact patch, other than desired tire

        CP_2 : Node
            One contact patch, other than desired tire and CP_1
        
        CP_3 : Node
            One contact patch, other than desired tire, CP_1, and CP_2
        
        tire : Tire
            Tire with desired inclination angle
        
        Returns
        -------
        float
            Inclination angle of given tire in degrees
        """
        cp_1 = np.array(CP_1.position)
        cp_2 = np.array(CP_2.position)
        cp_3 = np.array(CP_3.position)

        tire_pt_1 = np.array(tire.contact_patch.position)
        tire_pt_2 = np.array(tire.center_node.position)
        tire_pt_3 = np.array(tire.front_node.position)

        normal_1 = np.cross((cp_2 - cp_1), (cp_3 - cp_1))
        normal_2 = np.cross((tire_pt_2 - tire_pt_1), (tire_pt_3 - tire_pt_1))

        angle = np.arccos(np.linalg.norm(np.dot(normal_1, normal_2)) / (np.linalg.norm(normal_1) * np.linalg.norm(normal_2)))
        gamma_magnitude = (angle - np.pi / 2)

        if (np.sign(tire.direction[1]) == np.sign(tire.direction[2])):
            gamma_dir = -1
        else:
            gamma_dir = 1

        return gamma_magnitude * gamma_dir * 180 / np.pi

    def _update_state(self) -> None:
        """
        ## Update State

        Updates state dict

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.state["veh_CG_x"] = self.veh_CG[0]
        self.state["veh_CG_y"] = self.veh_CG[1]
        self.state["veh_CG_z"] = self.veh_CG[2]
        self.state["FL_gamma"] = self.FL_gamma
        self.state["FR_gamma"] = self.FR_gamma
        self.state["RL_gamma"] = self.RL_gamma
        self.state["RR_gamma"] = self.RR_gamma
        self.state["FL_delta"] = self.FL_delta
        self.state["FR_delta"] = self.FR_delta
        self.state["RL_delta"] = self.RL_delta
        self.state["RR_delta"] = self.RR_delta
        self.state["FL_caster"] = self.FL_caster
        self.state["FR_caster"] = self.FR_caster
        self.state["RL_caster"] = self.RL_caster
        self.state["RR_caster"] = self.RR_caster
        self.state["FL_kpi"] = self.FL_kpi
        self.state["FR_kpi"] = self.FR_kpi
        self.state["RL_kpi"] = self.RL_kpi
        self.state["RR_kpi"] = self.RR_kpi
        self.state["FL_mech_trail"] = self.FL_mech_trail
        self.state["FR_mech_trail"] = self.FR_mech_trail
        self.state["RL_mech_trail"] = self.RL_mech_trail
        self.state["RR_mech_trail"] = self.RR_mech_trail
        self.state["FL_scrub"] = self.FL_scrub
        self.state["FR_scrub"] = self.FR_scrub
        self.state["RL_scrub"] = self.RL_scrub
        self.state["RR_scrub"] = self.RR_scrub
        self.state["FL_FVIC_y"] = self.FL_FVIC[1]
        self.state["FR_FVIC_y"] = self.FR_FVIC[1]
        self.state["RL_FVIC_y"] = self.RL_FVIC[1]
        self.state["RR_FVIC_y"] = self.RR_FVIC[1]
        self.state["FL_FVIC_z"] = self.FL_FVIC[2]
        self.state["FR_FVIC_z"] = self.FR_FVIC[2]
        self.state["RL_FVIC_z"] = self.RL_FVIC[2]
        self.state["RR_FVIC_z"] = self.RR_FVIC[2]
        self.state["FL_SVIC_x"] = self.FL_SVIC[0]
        self.state["FR_SVIC_x"] = self.FR_SVIC[0]
        self.state["RL_SVIC_x"] = self.RL_SVIC[0]
        self.state["RR_SVIC_x"] = self.RR_SVIC[0]
        self.state["FL_SVIC_z"] = self.FL_SVIC[2]
        self.state["FR_SVIC_z"] = self.FR_SVIC[2]
        self.state["RL_SVIC_z"] = self.RL_SVIC[2]
        self.state["RR_SVIC_z"] = self.RR_SVIC[2]
        self.state["Fr_RC_y"] = self.Fr_RC[1]
        self.state["Fr_RC_z"] = self.Fr_RC[2]
        self.state["Rr_RC_y"] = self.Rr_RC[1]
        self.state["Rr_RC_z"] = self.Rr_RC[2]
        self.state["FL_spring_MR"] = self.FL_spring_MR
        self.state["FR_spring_MR"] = self.FR_spring_MR
        self.state["RL_spring_MR"] = self.RL_spring_MR
        self.state["RR_spring_MR"] = self.RR_spring_MR
        if self.Fr_stabar_MR:
            self.state["Fr_stabar_MR_rot"] = self.Fr_stabar_MR[0]
            self.state["Fr_stabar_MR_trans"] = self.Fr_stabar_MR[1]
        if self.Rr_stabar_MR:
            self.state["Rr_stabar_MR_rot"] = self.Rr_stabar_MR[0]
            self.state["Rr_stabar_MR_trans"] = self.Rr_stabar_MR[1]

    def _sprung_to_global(self, node: Node, align_axes: bool = True) -> Node:
        FL_cp = np.array(self.FL_quarter_car.tire.contact_patch.position)
        FR_cp = np.array(self.FR_quarter_car.tire.contact_patch.position)
        RL_cp = np.array(self.RL_quarter_car.tire.contact_patch.position)

        # Define planes
        FL_FR = FR_cp - FL_cp
        FL_RL = RL_cp - FL_cp

        normal_1 = np.cross(FL_FR, FL_RL)
        normal_2 = np.array([0, 0, 1])

        if abs(normal_1 / np.linalg.norm(normal_1)).all() == normal_2.all():
            return node

        # Rotation calcs
        rotation_axis = np.cross(normal_1, normal_2) / np.linalg.norm(np.cross(normal_1, normal_2))
        angle_mag = np.arccos(abs(np.dot(normal_1, normal_2)) / (np.linalg.norm(normal_1) * np.linalg.norm(normal_2)))
        trans_mat = np.array(rotation_matrix(unit_vec=rotation_axis, theta=-1 * angle_mag))
        
        node_trans = trans_mat @ node.position
        FL_cp_trans = trans_mat @ FL_cp
        FR_cp_trans = trans_mat @ FR_cp

        # Align axes
        if align_axes:
            x_offset = FL_cp_trans[0]
            y_offset = (FL_cp_trans[1] + FR_cp_trans[1]) / 2
            z_offset = FL_cp_trans[2]

            offset_vec = np.array([x_offset, y_offset, z_offset])

            node_trans = node_trans - offset_vec
        else:
            node_trans = node_trans

        return Node(position=node_trans)
    
    @property
    def FL_delta(self) -> float:
        """
        ## Front-Left Steered Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Steered angle of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Steered angle of the front-left tire in degrees
        """
        return self.FL_quarter_car.tire.delta

    @property
    def FR_delta(self) -> float:
        """
        ## Front-Right Steered Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Steered angle of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Steered angle of the front-right tire in degrees
        """
        return self.FR_quarter_car.tire.delta

    @property
    def RL_delta(self) -> float:
        """
        ## Rear-Left Steered Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Steered angle of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Steered angle of the rear-left tire in degrees
        """
        return self.RL_quarter_car.tire.delta

    @property
    def RR_delta(self) -> float:
        """
        ## Rear-Right Steered Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Steered angle of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Steered angle of the rear-right tire in degrees
        """
        return self.RR_quarter_car.tire.delta

    @property
    def FL_gamma(self) -> float:
        """
        ## Front-Left Inclination Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Inclination angle of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Inclination angle of front left tire in degrees
        """
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._gamma_calculation(CP_1=FR_cp, CP_2=RL_cp, CP_3=RR_cp, tire=self.FL_quarter_car.tire)

    @property
    def FR_gamma(self) -> float:
        """
        ## Front-Right Inclination Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Inclination angle of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Inclination angle of front right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._gamma_calculation(CP_1=FL_cp, CP_2=RL_cp, CP_3=RR_cp, tire=self.FR_quarter_car.tire)

    @property
    def RL_gamma(self) -> float:
        """
        ## Rear-Left Inclination Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Inclination angle of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Inclination angle of rear left tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._gamma_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RR_cp, tire=self.RL_quarter_car.tire)

    @property
    def RR_gamma(self) -> float:
        """
        ## Rear-Right Inclination Angle
        ##### Axis System: [X_{T}, Y_{T}, Z_{T}]
        ##### Coordinate System: [x_{T}, y_{T}, z_{T}]

        Inclination angle of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Inclination angle of rear right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch

        return self._gamma_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RL_cp, tire=self.RR_quarter_car.tire)

    @property
    def FL_caster(self) -> float:
        """
        ## Front-Left Caster
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Caster of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Caster of the front-left tire in degrees
        """
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._caster_calculation(CP_1=FR_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FL_quarter_car)

    @property
    def FR_caster(self) -> float:
        """
        ## Front-Right Caster
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Caster of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Caster of the front-right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._caster_calculation(CP_1=FL_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FR_quarter_car)

    @property
    def RL_caster(self) -> float:
        """
        ## Rear-Left Caster
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Caster of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Caster of the rear-left tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._caster_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RR_cp, quarter_car=self.RL_quarter_car)
    
    @property
    def RR_caster(self) -> float:
        """
        ## Rear-Right Caster
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Caster of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Caster of the rear-right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch

        return self._caster_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RL_cp, quarter_car=self.RR_quarter_car)

    @property
    def FL_kpi(self) -> float:
        """
        ## Front-Left KPI
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Kingpin inclination of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Kingpin inclination of the front-left tire in degrees
        """
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._kpi_calculation(CP_1=FR_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FL_quarter_car)

    @property
    def FR_kpi(self) -> float:
        """
        ## Front-Right KPI
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Kingpin inclination of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Kingpin inclination of the front-right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._kpi_calculation(CP_1=FL_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FR_quarter_car)

    @property
    def RL_kpi(self) -> float:
        """
        ## Rear-Left KPI
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Kingpin inclination of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Kingpin inclination of the rear-left tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._kpi_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RR_cp, quarter_car=self.RL_quarter_car)
    
    @property
    def RR_kpi(self) -> float:
        """
        ## Rear-Right KPI
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Kingpin inclination of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Kingpin inclination of the rear-right tire in degrees
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch

        return self._kpi_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RL_cp, quarter_car=self.RR_quarter_car)

    @property
    def FL_scrub(self) -> float:
        """
        ## Front-Left Scrub
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Scrub radius of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Scrub radius of the front-left tire
        """
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._scrub_calculation(CP_1=FR_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FL_quarter_car)

    @property
    def FR_scrub(self) -> float:
        """
        ## Front-Right Scrub
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Scrub radius of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Scrub radius of the front-right tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._scrub_calculation(CP_1=FL_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FR_quarter_car)

    @property
    def RL_scrub(self) -> float:
        """
        ## Rear-Left Scrub
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Scrub radius of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Scrub radius of the rear-left tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._scrub_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RR_cp, quarter_car=self.RL_quarter_car)
    
    @property
    def RR_scrub(self) -> float:
        """
        ## Rear-Right Scrub
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Scrub radius of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Scrub radius of the rear-right tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch

        return self._scrub_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RL_cp, quarter_car=self.RR_quarter_car)
    
    @property
    def FL_mech_trail(self) -> float:
        """
        ## Front-Left Mech Trail
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Mechanical trail of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Mechanical trail of the front-left tire
        """
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._mech_trail_calculation(CP_1=FR_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FL_quarter_car)

    @property
    def FR_mech_trail(self) -> float:
        """
        ## Front-Right Mech Trail
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Mechanical trail of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Mechanical trail of the front-right tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._mech_trail_calculation(CP_1=FL_cp, CP_2=RL_cp, CP_3=RR_cp, quarter_car=self.FR_quarter_car)

    @property
    def RL_mech_trail(self) -> float:
        """
        ## Rear-Left Mech Trail
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Mechanical trail of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Mechanical trail of the rear-left tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RR_cp = self.RR_quarter_car.tire.contact_patch

        return self._mech_trail_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RR_cp, quarter_car=self.RL_quarter_car)
    
    @property
    def RR_mech_trail(self) -> float:
        """
        ## Rear-Right Mech Trail
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Mechanical trail of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        float
            Mechanical trail of the rear-right tire
        """
        FL_cp = self.FL_quarter_car.tire.contact_patch
        FR_cp = self.FR_quarter_car.tire.contact_patch
        RL_cp = self.RL_quarter_car.tire.contact_patch

        return self._mech_trail_calculation(CP_1=FL_cp, CP_2=FR_cp, CP_3=RL_cp, quarter_car=self.RR_quarter_car)
    
    @property
    def veh_CG(self) -> Tuple[float, float, float]:
        """
        ## Vehicle CG
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]
        
        Vehicle CG position

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Vehicle CG position
        """
        
        global_veh_CG = self._sprung_to_global(node=self.sus_data.CG_node)
        
        return (global_veh_CG[0], global_veh_CG[1], global_veh_CG[2])

    @property
    def FL_FVIC(self) -> Tuple[float, float, float]:
        """
        ## Front-Left FVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Front-view instant center location of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Front-view instant center location of the front-left tire
        """
        upper_wishbone = self.FL_quarter_car.upper_wishbone
        lower_wishbone = self.FL_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        x_coord = self.FL_quarter_car.tire.contact_patch[0]

        A = np.array([[b1, c1],
                      [b2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - a1 * x_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - a2 * x_coord]])
        
        y, z = np.linalg.solve(a=A, b=B).T[0]

        global_FVIC = self._sprung_to_global(node=Node(position=[x_coord, y, z]))

        return (global_FVIC[0], global_FVIC[1], global_FVIC[2])

    @property
    def FR_FVIC(self) -> Tuple[float, float, float]:
        """
        ## Front-Right FVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Front-view instant center of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Front-view instant center of the front-right tire
        """
        upper_wishbone = self.FR_quarter_car.upper_wishbone
        lower_wishbone = self.FR_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        x_coord = self.FR_quarter_car.tire.contact_patch[0]

        A = np.array([[b1, c1],
                      [b2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - a1 * x_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - a2 * x_coord]])
        
        y, z = np.linalg.solve(a=A, b=B).T[0]

        global_FVIC = self._sprung_to_global(node=Node(position=[x_coord, y, z]))

        return (global_FVIC[0], global_FVIC[1], global_FVIC[2])
    
    @property
    def RL_FVIC(self) -> Tuple[float, float, float]:
        """
        ## Rear-Left FVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Front-view instant center of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Front-view instant center of the rear-left tire
        """
        upper_wishbone = self.RL_quarter_car.upper_wishbone
        lower_wishbone = self.RL_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        x_coord = self.RL_quarter_car.tire.contact_patch[0]

        A = np.array([[b1, c1],
                      [b2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - a1 * x_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - a2 * x_coord]])
        
        y, z = np.linalg.solve(a=A, b=B).T[0]

        global_FVIC = self._sprung_to_global(node=Node(position=[x_coord, y, z]))

        return (global_FVIC[0], global_FVIC[1], global_FVIC[2])

    @property
    def RR_FVIC(self) -> Tuple[float, float, float]:
        """
        ## Rear-Right FVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Front-view instant center of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Front-view instant center of the rear-right tire
        """
        upper_wishbone = self.RR_quarter_car.upper_wishbone
        lower_wishbone = self.RR_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        x_coord = self.RR_quarter_car.tire.contact_patch[0]

        A = np.array([[b1, c1],
                      [b2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - a1 * x_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - a2 * x_coord]])
        
        y, z = np.linalg.solve(a=A, b=B).T[0]

        global_FVIC = self._sprung_to_global(node=Node(position=[x_coord, y, z]))

        return (global_FVIC[0], global_FVIC[1], global_FVIC[2])
    
    @property
    def FL_SVIC(self) -> Tuple[float, float, float]:
        """
        ## Front-Left SVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Side-view instant center of the front-left tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Side-view instant center of the front-left tire
        """
        upper_wishbone = self.FL_quarter_car.upper_wishbone
        lower_wishbone = self.FL_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        y_coord = self.FL_quarter_car.tire.contact_patch[1]

        A = np.array([[a1, c1],
                      [a2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - b1 * y_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - b2 * y_coord]])
        
        try:
            x, z = np.linalg.solve(a=A, b=B).T[0]

        except np.linalg.LinAlgError:
            tire_center = np.array(self.FL_quarter_car.tire.center)
            SVIC_dir = np.array([c1, 0, -1 * a1]) / np.linalg.norm([c1, 0, -1 * a1])
            SVIC_dir *= -1 if SVIC_dir[0] < 0 else 1
            x, z = (tire_center + 1e9 * SVIC_dir)[[0, 2]]

        global_SVIC = self._sprung_to_global(node=Node(position=[x, y_coord, z]))

        return (global_SVIC[0], global_SVIC[1], global_SVIC[2])

    @property
    def FR_SVIC(self) -> Tuple[float, float, float]:
        """
        ## Front-Right SVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Side-view instant center of the front-right tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Side-view instant center of the front-right tire
        """
        upper_wishbone = self.FR_quarter_car.upper_wishbone
        lower_wishbone = self.FR_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        y_coord = self.FR_quarter_car.tire.contact_patch[1]

        A = np.array([[a1, c1],
                      [a2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - b1 * y_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - b2 * y_coord]])
        
        try:
            x, z = np.linalg.solve(a=A, b=B).T[0]

        except np.linalg.LinAlgError:
            tire_center = np.array(self.FR_quarter_car.tire.center)
            SVIC_dir = np.array([c1, 0, -1 * a1]) / np.linalg.norm([c1, 0, -1 * a1])
            SVIC_dir *= -1 if SVIC_dir[0] < 0 else 1
            x, z = (tire_center + 1e9 * SVIC_dir)[[0, 2]]

        global_SVIC = self._sprung_to_global(node=Node(position=[x, y_coord, z]))

        return (global_SVIC[0], global_SVIC[1], global_SVIC[2])
    
    @property
    def RL_SVIC(self) -> Tuple[float, float, float]:
        """
        ## Rear-Left SVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Side-view instant center of the rear-left tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Side-view instant center of the rear-left tire
        """
        upper_wishbone = self.RL_quarter_car.upper_wishbone
        lower_wishbone = self.RL_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        y_coord = self.RL_quarter_car.tire.contact_patch[1]

        A = np.array([[a1, c1],
                      [a2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - b1 * y_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - b2 * y_coord]])
        
        try:
            x, z = np.linalg.solve(a=A, b=B).T[0]
            
        except np.linalg.LinAlgError:
            tire_center = np.array(self.RL_quarter_car.tire.center)
            SVIC_dir = np.array([c1, 0, -1 * a1]) / np.linalg.norm([c1, 0, -1 * a1])
            SVIC_dir *= -1 if SVIC_dir[0] < 0 else 1
            x, z = (tire_center + 1e9 * SVIC_dir)[[0, 2]]

        global_SVIC = self._sprung_to_global(node=Node(position=[x, y_coord, z]))

        return (global_SVIC[0], global_SVIC[1], global_SVIC[2])

    @property
    def RR_SVIC(self) -> Tuple[float, float, float]:
        """
        ## Rear-Right SVIC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Side-view instant center of the rear-right tire

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Side-view instant center of the rear-right tire
        """
        upper_wishbone = self.RR_quarter_car.upper_wishbone
        lower_wishbone = self.RR_quarter_car.lower_wishbone

        a1, b1, c1, x0_1, y0_1, z0_1 = upper_wishbone.plane
        a2, b2, c2, x0_2, y0_2, z0_2 = lower_wishbone.plane
        y_coord = self.RR_quarter_car.tire.contact_patch[1]

        A = np.array([[a1, c1],
                      [a2, c2]])
        
        B = np.array([[(a1 * x0_1 + b1 * y0_1 + c1 * z0_1) - b1 * y_coord],
                      [(a2 * x0_2 + b2 * y0_2 + c2 * z0_2) - b2 * y_coord]])
        
        try:
            x, z = np.linalg.solve(a=A, b=B).T[0]

        except np.linalg.LinAlgError:
            tire_center = np.array(self.RR_quarter_car.tire.center)
            SVIC_dir = np.array([c1, 0, -1 * a1]) / np.linalg.norm([c1, 0, -1 * a1])
            SVIC_dir *= -1 if SVIC_dir[0] < 0 else 1
            x, z = (tire_center + 1e9 * SVIC_dir)[[0, 2]]

        global_SVIC = self._sprung_to_global(node=Node(position=[x, y_coord, z]))

        return (global_SVIC[0], global_SVIC[1], global_SVIC[2])
    
    @property
    def Fr_RC(self) -> Tuple[float, float, float]:
        """
        ## Front RC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Roll center of the front axle

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Calculates position vector of front roll center
        """
        FL_cp = self._sprung_to_global(self.FL_quarter_car.tire.contact_patch)
        FR_cp = self._sprung_to_global(self.FR_quarter_car.tire.contact_patch)

        left_link = Link(inboard_node=Node(position=self.FL_FVIC), outboard_node=FL_cp)
        right_link = Link(inboard_node=Node(position=self.FR_FVIC), outboard_node=FR_cp)

        rc_node = left_link.yz_intersection(link=right_link)

        return (rc_node[0], rc_node[1], rc_node[2])

    @property
    def Rr_RC(self) -> Tuple[float, float, float]:
        """
        ## Rear RC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Roll center of the rear axle

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Calculates position vector of rear roll center
        """
        RL_cp = self._sprung_to_global(self.RL_quarter_car.tire.contact_patch)
        RR_cp = self._sprung_to_global(self.RR_quarter_car.tire.contact_patch)

        left_link = Link(inboard_node=Node(position=self.RL_FVIC), outboard_node=RL_cp)
        right_link = Link(inboard_node=Node(position=self.RR_FVIC), outboard_node=RR_cp)

        rc_node = left_link.yz_intersection(link=right_link)

        return (rc_node[0], rc_node[1], rc_node[2])
    
    @property
    def left_PC(self) -> Tuple[float, float, float]:
        """
        ## Left PC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Left kinematic pitch center

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Calculates position vector of left kinematic pitch center
        """
        FL_cp = self._sprung_to_global(self.FL_quarter_car.tire.contact_patch)
        RL_cp = self._sprung_to_global(self.RL_quarter_car.tire.contact_patch)

        front_link = Link(inboard_node=Node(position=self.FL_SVIC), outboard_node=FL_cp)
        rear_link = Link(inboard_node=Node(position=self.RL_SVIC), outboard_node=RL_cp)

        # pc_node = front_link.xz_intersection(link=rear_link)
    
        # return pc_node.position
    
        avg_cp = (self.FL_quarter_car.tire.contact_patch + \
                  self.FR_quarter_car.tire.contact_patch + \
                  self.RL_quarter_car.tire.contact_patch + \
                  self.RR_quarter_car.tire.contact_patch) / 4
        
        return (avg_cp[0], avg_cp[1], avg_cp[2])

    @property
    def right_PC(self) -> Tuple[float, float, float]:
        """
        ## Right PC
        ##### Axis System: [X_{E}, Y_{E}, Z_{E}]
        ##### Coordinate System: [x_{E}, y_{E}, z_{E}]

        Right kinematic pitch center

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float, float]
            Calculates position vector of right kinematic pitch center
        """
        FR_cp = self._sprung_to_global(self.FR_quarter_car.tire.contact_patch)
        RR_cp = self._sprung_to_global(self.RR_quarter_car.tire.contact_patch)

        front_link = Link(inboard_node=Node(position=self.FR_SVIC), outboard_node=FR_cp)
        rear_link = Link(inboard_node=Node(position=self.RR_SVIC), outboard_node=RR_cp)

        # pc_node = front_link.xz_intersection(link=rear_link)

        # return pc_node.position

        avg_cp = (self.FL_quarter_car.tire.contact_patch + \
                  self.FR_quarter_car.tire.contact_patch + \
                  self.RL_quarter_car.tire.contact_patch + \
                  self.RR_quarter_car.tire.contact_patch) / 4
        
        return (avg_cp[0], avg_cp[1], avg_cp[2])

    @property
    def FL_spring_MR(self) -> float:
        """
        ## Front-Left Spring MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratio of the front-left spring, defined disp(wheel)/disp(spring)

        Parameters
        ----------
        None

        Returns
        -------
        float
            motion ratio of the front-left spring
        """

        # Apply small displacement
        self.FL_quarter_car._jounce_persistent(jounce=-0.001)
        spring_length_1 = self.FL_quarter_car.push_pull_rod.spring.length

        # Must displace by 0.002 to reach 0.001 relative to the original state
        self.FL_quarter_car._jounce_persistent(jounce=0.002)
        spring_length_2 = self.FL_quarter_car.push_pull_rod.spring.length

        # Return corner to original state (-0.001)
        self.FL_quarter_car._jounce_persistent(jounce=-0.001)

        try:
            return abs((0.002) / (spring_length_1 - spring_length_2))
        except:
            return 1.25
        
    @property
    def FR_spring_MR(self) -> float:
        """
        ## Front-Right Spring MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratio of the front-right spring, defined disp(wheel)/disp(spring)

        Parameters
        ----------
        None

        Returns
        -------
        float
            Motion ratio of the front-right spring
        """

        # Apply small displacement
        self.FR_quarter_car._jounce_persistent(jounce=-0.001)
        spring_length_1 = self.FR_quarter_car.push_pull_rod.spring.length

        # Must displace by 0.002 to reach 0.001 relative to the original state
        self.FR_quarter_car._jounce_persistent(jounce=0.002)
        spring_length_2 = self.FR_quarter_car.push_pull_rod.spring.length

        # Return corner to original state (-0.001)
        self.FR_quarter_car._jounce_persistent(jounce=-0.001)

        try:
            return abs((0.002) / (spring_length_1 - spring_length_2))
        except:
            return 1.25
        
    @property
    def RL_spring_MR(self) -> float:
        """
        ## Rear-Left Spring MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratio of the rear-left spring, defined disp(wheel)/disp(spring)

        Parameters
        ----------
        None

        Returns
        -------
        float
            Motion ratio of the rear-left spring
        """

        # Apply small displacement
        self.RL_quarter_car._jounce_persistent(jounce=-0.001)
        spring_length_1 = self.RL_quarter_car.push_pull_rod.spring.length

        # Must displace by 0.002 to reach 0.001 relative to the original state
        self.RL_quarter_car._jounce_persistent(jounce=0.002)
        spring_length_2 = self.RL_quarter_car.push_pull_rod.spring.length

        # Return corner to original state (-0.001)
        self.RL_quarter_car._jounce_persistent(jounce=-0.001)

        try:
            return abs((0.002) / (spring_length_1 - spring_length_2))
        except:
            return 1.4
        
    @property
    def RR_spring_MR(self) -> float:
        """
        ## Rear-Right Spring MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratio of the rear-right spring, defined disp(wheel)/disp(stabar arm)

        Parameters
        ----------
        None

        Returns
        -------
        float
            Motion ratio of the rear-right spring
        """

        # Apply small displacement
        self.RR_quarter_car._jounce_persistent(jounce=-0.001)
        spring_length_1 = self.RR_quarter_car.push_pull_rod.spring.length

        # Must displace by 0.002 to reach 0.001 relative to the original state
        self.RR_quarter_car._jounce_persistent(jounce=0.002)
        spring_length_2 = self.RR_quarter_car.push_pull_rod.spring.length

        # Return corner to original state (-0.001)
        self.RR_quarter_car._jounce_persistent(jounce=-0.001)

        try:
            return abs((0.002) / (spring_length_1 - spring_length_2))
        except:
            return 1.4
        
    
    @property
    def Fr_stabar_MR(self) -> Tuple[float, float]:
        """
        ## Front Stabar MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratios of the front anti-roll bar, defined [roll/ang(stabar arm), disp(wheel)/disp(stabar arm)]

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float]
            Motion ratios of the front anti-roll bar, in the form [roll/ang, disp(wheel)/disp(stabar arm)]
        """
        
        if self.sus_data.Fr_stabar:
            # Make copies
            self_copy_1 = deepcopy(self)
            self_copy_2 = deepcopy(self)

            Fr_stabar_1 = cast(Stabar, self_copy_1.sus_data.Fr_stabar)
            Fr_stabar_2 = cast(Stabar, self_copy_2.sus_data.Fr_stabar)

            # Apply small rotation
            self_copy_1.roll(roll=-0.001, update_state=False)
            stabar_rot_1 = Fr_stabar_1.rotation * 180 / np.pi

            self_copy_2.roll(roll=0.001, update_state=False)
            stabar_rot_2 = Fr_stabar_2.rotation * 180 / np.pi

            angle_mr = abs((0.002) / (stabar_rot_2 - stabar_rot_1))

            # Make new copies for jounce
            self_copy_1 = deepcopy(self)
            self_copy_2 = deepcopy(self)

            Fr_stabar_1 = cast(Stabar, self_copy_1.sus_data.Fr_stabar)
            Fr_stabar_2 = cast(Stabar, self_copy_2.sus_data.Fr_stabar)

            # Apply small displacement
            self_copy_1.RL_quarter_car._jounce_persistent(jounce=-0.001)
            stabar_arm_pos_1 = Fr_stabar_1.left_arm.outboard_node[2]

            # Must displace by 0.002 to reach 0.001 relative to the original state
            self_copy_2.RL_quarter_car._jounce_persistent(jounce=0.001)
            stabar_arm_pos_2 = Fr_stabar_2.left_arm.outboard_node[2]

            trans_mr = abs((0.002) / (stabar_arm_pos_2 - stabar_arm_pos_1))

            return (angle_mr, trans_mr)

        else:
            return (0, 0)

    @property
    def Rr_stabar_MR(self) -> Tuple[float, float]:
        """
        ## Rear Stabar MR
        ##### Axis System: [X_{V}, Y_{V}, Z_{V}]
        ##### Coordinate System: [x_{V}, y_{V}, z_{V}]

        Motion ratios of the rear anti-roll bar, defined [roll/ang(stabar arm), disp(wheel)/disp(stabar arm)]

        Parameters
        ----------
        None

        Returns
        -------
        Tuple[float, float]
            Motion ratios of the rear anti-roll bar, in the form [roll/ang, disp(wheel)/disp(stabar arm)]
        """
        if self.sus_data.Rr_stabar:
            # Make copies
            self_copy_1 = deepcopy(self)
            self_copy_2 = deepcopy(self)

            Rr_stabar_1 = cast(Stabar, self_copy_1.sus_data.Rr_stabar)
            Rr_stabar_2 = cast(Stabar, self_copy_2.sus_data.Rr_stabar)

            # Apply small rotation
            self_copy_1.roll(roll=-0.001, update_state=False)
            stabar_rot_1 = Rr_stabar_1.rotation * 180 / np.pi

            self_copy_2.roll(roll=0.001, update_state=False)
            stabar_rot_2 = Rr_stabar_2.rotation * 180 / np.pi

            try:
                angle_mr = abs((0.002) / (stabar_rot_2 - stabar_rot_1))
            except:
                angle_mr = 0.4
            # Make new copies for jounce
            self_copy_1 = deepcopy(self)
            self_copy_2 = deepcopy(self)

            Rr_stabar_1 = cast(Stabar, self_copy_1.sus_data.Rr_stabar)
            Rr_stabar_2 = cast(Stabar, self_copy_2.sus_data.Rr_stabar)

            # Apply small displacement
            self_copy_1.RL_quarter_car._jounce_persistent(jounce=-0.001)
            stabar_arm_pos_1 = Rr_stabar_1.left_arm.outboard_node[2]

            # Must displace by 0.002 to reach 0.001 relative to the original state
            self_copy_2.RL_quarter_car._jounce_persistent(jounce=0.001)
            stabar_arm_pos_2 = Rr_stabar_2.left_arm.outboard_node[2]

            try:
                trans_mr = abs((0.002) / (stabar_arm_pos_2 - stabar_arm_pos_1))
            except:
                trans_mr = 11
                
            return (angle_mr, trans_mr)

        else:
            return (0, 0)