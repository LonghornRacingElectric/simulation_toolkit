from vehicle_model.suspension_model.suspension_elements.secondary_elements.steering_link import SteeringLink
from vehicle_model.suspension_model.suspension_elements.secondary_elements.push_pull_rod import PushPullRod
from vehicle_model.suspension_model.suspension_elements.secondary_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements.secondary_elements.kingpin import Kingpin
from vehicle_model.suspension_model.suspension_elements.tertiary_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
from vehicle_model.suspension_model.assets.plotter import Plotter
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class DoubleWishbone:
    """
    ## Double Wishbone

    Double wishbone object

    Parameters
    ----------
    inboard_points : Sequence[Sequence[float]]
        Array containing all inboard coordinates of the double wishbone
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    outboard_points : Sequence[Sequence[float]]
        Array containing all outboard coordinates of the double wishbone
        - Order: [[Upper Fore], [Upper Aft], [Lower Fore], [Lower Aft], [Tie Rod], [Push/Pull Rod]]
    bellcrank_params : Sequence[Sequence[float]]
        Relevant bellcrank to shock parameters
        - Order: [[Pivot], [Pivot Direction], [Shock Outboard], [Shock Inboard]]
    spring_rate : float
        Rate of spring in N/m
    weight : float
        Weight of the corner in N
    cg : CG
        CG object defining the center of gravity
    upper : bool
        True if push/pull rod mounts to upper wishbone
    contact_patch : Sequence[float]
        Coordinates of contact patch
    inclination_angle : float
        Inclination angle of tire in radians
    toe : float
        Static toe angle of tire in radians
        - Uses same sign convention as slip angle, NOT symmetric
    tire_radius : float
        Radius of tire
        - Must use same units as suspension coordinates above
    tire_width : float
        Width of tire
        - Must use same units as suspension coordinates above
    show_ICs : bool
        Toggle visibility of double wishbone vs swing arms
    """
    def __init__(
            self,
            inboard_points: Sequence[Sequence[float]],
            outboard_points: Sequence[Sequence[float]],
            bellcrank_params: Sequence[Sequence[float]],
            spring_rate: float,
            weight: float,
            cg: CG,
            upper: bool,
            contact_patch: Sequence[float],
            inclination_angle: float,
            toe: float,
            tire_radius: float,
            tire_width: float,
            show_ICs: bool) -> None:
        
        # Initialize travel limits
        self.max_jounce: float | None = None
        self.max_rebound: float | None = None
        self.max_steer: float | None = None

        # Initialize load parameters
        self.spring_rate = spring_rate
        self.weight = weight
        
        # Initialize state and plotter
        self.total_jounce:float = 0

        self.heave_jounce: float = 0
        self.roll_jounce: float = 0
        self.pitch_jounce: float = 0

        self.steered_angle = 0
        self.induced_steer = 0

        self.sus_plotter = Plotter()
    
        # Define all points
        upper_fore_inboard: Node = Node(position=inboard_points[0])
        upper_aft_inboard: Node = Node(position=inboard_points[1])
        lower_fore_inboard: Node = Node(position=inboard_points[2])
        lower_aft_inboard: Node = Node(position=inboard_points[3])
        tie_inboard: Node = Node(position=inboard_points[4])

        upper_fore_outboard: Node = Node(position=outboard_points[0])
        upper_aft_outboard: Node = upper_fore_outboard
        lower_fore_outboard: Node = Node(position=outboard_points[2])
        lower_aft_outboard: Node = lower_fore_outboard
        tie_outboard: Node = Node(position=outboard_points[4])

        self.cg = cg

        # Define all links
        self.upper_fore_link: Link = Link(inboard=upper_fore_inboard, outboard=upper_fore_outboard)
        self.upper_aft_link: Link = Link(inboard=upper_aft_inboard, outboard=upper_aft_outboard)
        self.lower_fore_link: Link = Link(inboard=lower_fore_inboard, outboard=lower_fore_outboard)
        self.lower_aft_link: Link = Link(inboard=lower_aft_inboard, outboard=lower_aft_outboard)

        # Define high-level components
        self.upper_wishbone: Wishbone = Wishbone(fore_link=self.upper_fore_link, aft_link=self.upper_aft_link)
        self.lower_wishbone: Wishbone = Wishbone(fore_link=self.lower_fore_link, aft_link=self.lower_aft_link)
        self.kingpin: Kingpin = Kingpin(lower_node=lower_fore_outboard, upper_node=upper_fore_outboard)
        self.steering_link: SteeringLink = SteeringLink(inboard=tie_inboard, outboard=tie_outboard, kingpin=self.kingpin)

        # Define unsprung parameters
        self.upper_outboard: Node = upper_fore_outboard
        self.lower_outboard: Node = lower_fore_outboard
        self.tie_outboard: Node = tie_outboard
        self.contact_patch: Node = Node(position=contact_patch)
        self.tire: Tire = Tire(contact_patch=self.contact_patch, 
                               kingpin=self.kingpin, 
                               static_gamma=inclination_angle, 
                               static_toe=toe, 
                               radius=tire_radius, 
                               width=tire_width)

        # Define instant centers
        self.FVIC = Node(position=self.FVIC_position)
        self.FVIC_link = Link(inboard=self.FVIC, outboard=self.contact_patch)
        self.SVIC = Node(position=self.SVIC_position)
        self.SVIC_link = Link(inboard=self.SVIC, outboard=self.contact_patch)

        # Define force application points
        self.FV_FAP = Node(position=self.FV_FAP_position)
        self.SV_FAP = Node(position=self.SV_FAP_position)

        # Define push/pull rod
        rod_inboard = Node(position=inboard_points[5])
        rod_outboard = Node(position=outboard_points[5])
        bellcrank_pivot = Node(position=bellcrank_params[0])
        bellcrank_direction = bellcrank_params[1]
        shock_outboard = Node(position=bellcrank_params[2])
        shock_inboard = Node(position=bellcrank_params[3])
        
        self.rod = PushPullRod(inboard=rod_inboard, 
                               outboard=rod_outboard,
                               upper=upper,
                               bellcrank=True,
                               bellcrank_pivot=bellcrank_pivot,
                               bellcrank_direction=bellcrank_direction,
                               shock_outboard=shock_outboard,
                               shock_inboard=shock_inboard)

        # Track pushrod location for transformations later
        self.upper = upper

        # Cache unsprung geometry
        self._fixed_unsprung_geom()

        # Create function for motion ratio
        jounce_sweep = np.linspace(-0.0254 * 5, 0.0254 * 5, 100)
        jounce_interval = 0.0254 * 2 * 5 / 100

        motion_ratio_lst = []
        for jounce in jounce_sweep:
            upper = jounce + jounce_interval
            self.jounce(jounce=jounce)
            spring_pos_0 = self.rod.spring_damper_length
            self.jounce(jounce=upper)
            spring_pos_1 = self.rod.spring_damper_length

            motion_ratio = jounce_interval / (spring_pos_0 - spring_pos_1)
            motion_ratio_lst.append(motion_ratio)
        
        self.motion_ratio_function = CubicSpline(x=jounce_sweep, y=motion_ratio_lst)
        
        # Reset jounce
        self.jounce(jounce=0)

        # Create function for wheelrate
        wheelrate_lst = [self.spring_rate / MR**2 for MR in motion_ratio_lst]
        # wheelrate_lst = [self.spring_rate / MR**2 for MR in [1.3 for x in motion_ratio_lst]]
        self.wheelrate_function = CubicSpline(x=jounce_sweep, y=wheelrate_lst)

        # plt.plot(jounce_sweep, self.motion_ratio_function(jounce_sweep))
        # plt.show()

        # plt.plot(jounce_sweep, self.wheelrate_function(jounce_sweep))
        # plt.show()

        # Plotting
        if not show_ICs:
            self.elements = [self.upper_wishbone, self.lower_wishbone, self.rod, self.kingpin, self.steering_link, self.tire]
        elif max(np.abs(self.SVIC.position)) < 50:
            self.elements = [self.kingpin, self.steering_link, self.rod, self.tire, self.FVIC, self.SVIC, self.FVIC_link, self.SVIC_link, self.FV_FAP, self.SV_FAP]
        else:
            # self.elements = [self.kingpin, self.steering_link, self.rod, self.tire, self.FVIC, self.FVIC_link, self.FV_FAP, self.SV_FAP]
            self.elements = [self.kingpin, self.steering_link, self.tire, self.FVIC_link, self.FV_FAP, self.SV_FAP]
        
        # Rotations
        if max(np.abs(self.SVIC.position)) < 50:
            self.all_elements = [self.upper_wishbone, self.lower_wishbone, self.rod, self.steering_link, self.tire, self.FVIC, self.SVIC, self.FV_FAP, self.SV_FAP]
        else:
            self.all_elements = [self.upper_wishbone, self.lower_wishbone, self.rod, self.steering_link, self.tire, self.FVIC, self.FV_FAP, self.SV_FAP]
        # self.all_elements = [self.upper_wishbone, self.lower_wishbone, self.steering_link, self.tire]

    def _fixed_unsprung_geom(self) -> None:
        """
        ## Fixed Unsprung Geometry

        Calculates distance constraints imposed by unsprung geometry

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        # Distance constraints
        self.cp_to_lower = np.linalg.norm(self.contact_patch.position - self.lower_outboard.position)
        self.cp_to_upper = np.linalg.norm(self.contact_patch.position - self.upper_outboard.position)
        self.cp_to_tie = np.linalg.norm(self.contact_patch.position - self.tie_outboard.position)

        # Contact patch to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        cp_pos_shifted = self.contact_patch.position - self.lower_outboard.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.cp_to_kingpin = np.matmul(y_rot, np.matmul(x_rot, cp_pos_shifted))

    def _jounce_resid_func(self, x: Sequence[float], jounce: float) -> Sequence[float]:
        """
        ## Jounce Residual Function

        Residual function for jounce calculation convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution Guess
        jounce : float
            Desired jounce value

        Returns
        -------
        Sequence[float]
            Residuals
        """
        upper_wishbone_rot = x[0]
        lower_wishbone_rot = x[1]

        # Apply wishbone rotations
        self.upper_wishbone.rotate(angle=upper_wishbone_rot)
        self.lower_wishbone.rotate(angle=lower_wishbone_rot)

        # Calculate contact patch under jounce condition
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        cp_pos = np.matmul(y_rot, np.matmul(x_rot, self.cp_to_kingpin)) + self.lower_outboard.position

        # Geometry constraints
        cp_to_lower = np.linalg.norm(cp_pos - self.lower_outboard.position)
        cp_to_upper = np.linalg.norm(cp_pos - self.upper_outboard.position)
        offset = cp_pos[2] - jounce

        # Set global contact patch position
        self.contact_patch.position = cp_pos

        return [(cp_to_lower - self.cp_to_lower) + offset, (cp_to_upper - self.cp_to_upper) + offset]

    def _jounce_induced_steer_resid_func(self, x : Sequence[float]) -> Sequence[float]:
        """
        ## Jounce-Induced Steer Residual Function

        Residual function for jounce-induced steer calculation convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution guess

        Returns
        -------
        Sequence[float]
            Residuals
        """
        induced_steer = x[0]

        # Apply induced steering rotation
        self.steering_link.rotate(induced_steer)

        residual_length = self.steering_link.length - self.steering_link.initial_length

        return residual_length

    def jounce(self, jounce: float = 0, heave_jounce: float = 0, roll_jounce: float = 0, pitch_jounce: float = 0) -> None:
        """
        ## Jounce

        Updates double wishbone geometry for given vertical travel of the contact patch (jounce)

        Parameters
        ----------
        jounce : float, optional
            Vertical travel of the contact patch, by default 0
        heave_jounce : float, optional
            Vertical travel of the contact patch due to heave, by default 0
        roll_jounce : float, optional
            Vertical travel of the contact patch due to roll, by default 0
        pitch_jounce : float, optional
            Vertical travel of the contact patch due to pitch, by default 0
        
        Returns
        -------
        None
        """
        if jounce:
            self.heave_jounce = 0
            self.roll_jounce = 0
            self.pitch_jounce = 0
            
        if heave_jounce:
            self.heave_jounce = heave_jounce
        if roll_jounce:
            self.roll_jounce = roll_jounce
        if pitch_jounce:
            self.pitch_jounce = pitch_jounce

        self.total_jounce = jounce + self.heave_jounce + self.roll_jounce + self.pitch_jounce

        if self.total_jounce:
            wishbone_angles = fsolve(self._jounce_resid_func, [0, 0], args=(self.total_jounce))
        else:
            wishbone_angles = [0, 0]

        self.upper_wishbone.rotate(wishbone_angles[0])
        self.lower_wishbone.rotate(wishbone_angles[1])

        induced_steer = fsolve(self._jounce_induced_steer_resid_func, [0])
        self.steering_link.rotate(induced_steer[0])
        
        # Set jounce-induced steer in tire
        self.tire.induced_steer = induced_steer[0]

        # Apply transformation to push/pull rod
        if self.upper:
            self.rod.rotate_rod(axis=self.upper_wishbone.direction, origin=self.upper_wishbone.fore_link.inboard_node, angle=wishbone_angles[0])
        else:
            self.rod.rotate_rod(axis=self.lower_wishbone.direction, origin=self.lower_wishbone.fore_link.inboard_node, angle=wishbone_angles[1])
        self.rod.update()

        self.induced_steer = induced_steer[0]
        self.FVIC.position = self.FVIC_position
        self.SVIC.position = self.SVIC_position

        self.FV_FAP.position = self.FV_FAP_position
        self.SV_FAP.position = self.SV_FAP_position
    
    # def jounce_step(self, step: float):
    #     wishbone_angles = fsolve(self._jounce_resid_func, [0, 0], args=(self.current_jounce))

    #     self.upper_wishbone.rotate(wishbone_angles[0])
    #     self.lower_wishbone.rotate(wishbone_angles[1])

    #     induced_steer = fsolve(self._jounce_induced_steer_resid_func, [0])
    #     self.steering_link.rotate(induced_steer[0])
        
    #     # Set jounce-induced steer in tire
    #     self.tire.induced_steer = induced_steer[0]

    #     self.induced_steer = induced_steer[0]
    #     self.FVIC.position = self.FVIC_position
    #     self.SVIC.position = self.SVIC_position

    def _steer_resid_func(self, x: Sequence[float]) -> Sequence[float]:
        """
        ## Steer Residual Function

        Residual function for steer calculation convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution guess

        Returns
        -------
        Sequence[float]
            Residuals
        """
        steer_angle = x[0]

        self.steering_link.rotate(steer_angle)
        residual_length = self.steering_link.length - self.steering_link.initial_length

        return [residual_length]
    
    def steer(self, steer: float) -> None:
        """
        ## Steer

        Steers double wishbone

        Parameters
        ----------
        steer : float
            Lateral rack translation
        
        Returns
        -------
        None
        """
        self.steering_link.inboard_node.position[1] = self.steering_link.inboard_node.initial_position[1] + steer

        angle = fsolve(self._steer_resid_func, [0])

        self.tire.induced_steer = angle[0] + self.induced_steer

        self.steered_angle = steer
    
    @property
    def motion_ratio(self) -> float:
        """
        ## Motion Ratio

        Motion Ratio attribute under current jounce condition

        Returns
        -------
        float
            Motion ratio under current jounce condition
        """
        return self.motion_ratio_function(self.total_jounce)

    @property
    def wheelrate(self) -> float:
        """
        ## Wheelrate

        Calculates wheelrate under current jounce condition

        Returns
        -------
        float
            Wheelrate under current jounce condition
        """
        return self.spring_rate / self.motion_ratio**2
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply
            - Takes the form [x_shift, y_shift, z_shift]
        """
        for element in self.all_elements:
            element.translate(translation=translation)
    
    def flatten_rotate(self, angle: Sequence[float]) -> None:
        """
        ## Flatten Rotate

        Rotates all children
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        angle : Sequence[float]
            Angles of rotation in radians [x_rot, y_rot, z_rot]
        """
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)

        # Adjust force application points for rotation
        self.FV_FAP.position = self.FV_FAP_position
        self.SV_FAP.position = self.SV_FAP_position

    @property
    def lateral_arm(self) -> float:
        """
        ## Lateral Arm

        Calculates lateral distance between contact patch and center of gravity (cg)

        Returns
        -------
        float
            Lateral distance between contact patch and cg
        """
        lateral_arm = abs(self.contact_patch.position[1] - self.cg.position[1])

        return lateral_arm
    
    @property
    def longitudinal_arm(self) -> float:
        """
        ## Longitudinal Arm

        Calculates longitudinal distance between contact patch and center of gravity (cg)

        Returns
        -------
        float
            Longitudinal distance between contact patch and cg
        """
        longitudinal_arm = abs(self.contact_patch.position[0] - self.cg.position[0])

        return longitudinal_arm
    
    @property
    def FVIC_position(self) -> Sequence[float]:
        """
        ## FVIC Position

        Calculates position of front-view instance center

        Returns
        -------
        Sequence[float]
            Coordinates of front-view instance center
        """
        upper_plane = self.upper_wishbone.plane
        lower_plane = self.lower_wishbone.plane
        x = self.contact_patch.position[0]

        if (upper_plane[1] / upper_plane[2]) == (lower_plane[1] / lower_plane[2]):
            return [x, -1 * 1e9 * np.sign(self.contact_patch.position[1]), 0]

        a = np.array(
            [
                [upper_plane[1], upper_plane[2]],
                [lower_plane[1], lower_plane[2]]
            ])
        
        b = np.array(
            [
                [upper_plane[0] * (upper_plane[3] - x) + upper_plane[1] * upper_plane[4] + upper_plane[2] * upper_plane[5]],
                [lower_plane[0] * (lower_plane[3] - x) + lower_plane[1] * lower_plane[4] + lower_plane[2] * lower_plane[5]]
            ])

        soln = np.linalg.solve(a=a, b=b)

        y = soln[0][0]
        z = soln[1][0]

        return [x, y, z]
    
    @property
    def SVIC_position(self) -> Sequence[float]:
        """
        ## SVIC Position

        Calculates position of side-view instance center

        Returns
        -------
        Sequence[float]
            Coordinates of side-view instance center
        """
        upper_plane = self.upper_wishbone.plane
        lower_plane = self.lower_wishbone.plane
        y = self.contact_patch.position[1]

        if (upper_plane[0] / upper_plane[2]) == (lower_plane[0] / lower_plane[2]):
            if self.contact_patch.position[0] == 0:
                return [-1 * 1e9, y, 0]
            else:
                return [-1 * 1e9 * np.sign(self.contact_patch.position[0]), y, 0]

        a = np.array(
            [
                [upper_plane[0], upper_plane[2]],
                [lower_plane[0], lower_plane[2]]
            ])
        
        b = np.array(
            [
                [upper_plane[1] * (upper_plane[4] - y) + upper_plane[0] * upper_plane[3] + upper_plane[2] * upper_plane[5]],
                [lower_plane[1] * (lower_plane[4] - y) + lower_plane[0] * lower_plane[3] + lower_plane[2] * lower_plane[5]]
            ])

        soln = np.linalg.solve(a=a, b=b)

        x = soln[0][0]
        z = soln[1][0]

        return [x, y, z]

    @property
    def FV_FAP_position(self) -> Sequence[float]:
        """
        ## Front-view force application point height

        Calculates the position of the front-view force application point

        Returns
        -------
            Height of the front-view force application point
        """

        dir_yz = self.FVIC_link.inboard_node.position - self.FVIC_link.outboard_node.position
        z = (dir_yz[2] / dir_yz[1]) * (self.cg.position[1] - self.FVIC_link.outboard_node.position[1]) + self.FVIC_link.outboard_node.position[2]

        x = self.FVIC_link.outboard_node.position[0]
        y = self.cg.position[1]

        return [x, y, z]
    
    @property
    def SV_FAP_position(self) -> float:
        """
        ## Side-view force application point height

        Calculates the height of the side-view force application point

        Returns
        -------
            Height of the side-view force application point
        """

        dir_xz = self.SVIC_link.inboard_node.position - self.SVIC_link.outboard_node.position
        z = (dir_xz[2] / dir_xz[0]) * (self.cg.position[0] - self.SVIC_link.outboard_node.position[0]) + self.SVIC_link.outboard_node.position[2]

        x = self.cg.position[0]
        y = self.SVIC_link.outboard_node.position[1]
        
        return [x, y, z]

    @property
    def caster(self) -> float:
        """
        ## Caster

        Calculates caster

        Returns
        -------
        float
            Caster angle in radians
        """
        return self.kingpin.normalized_transform()[1]

    @property
    def kpi(self) -> float:
        """
        ## KPI

        Calculates kingpin inclination

        Returns
        -------
        float
            Kingpin inclination angle in radians
        """
        return self.kingpin.normalized_transform()[0]

    @property
    def toe(self) -> float:
        """
        ## Toe

        Calculates toe angle

        Returns
        -------
        float
            Toe angle in radians
        """
        return self.tire.induced_steer

    @property
    def inclination_angle(self) -> float:
        """
        ## Inclination Angle

        Calculates inclination angle

        Returns
        -------
        float
            Inclination angle in radians
        """
        # This only works because of the way I set up the direction vectors
        # You'll likely run into sign issues if applying this elsewhere
        vec_a = np.array(self.tire.direction)
        gamma = np.arctan(vec_a[2] / (np.sqrt(vec_a[0]**2 + vec_a[1]**2)))

        return gamma
    
    def plot_elements(self, plotter):
        """
        ## Plot Elements

        Plots all child elements

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        for element in self.elements:
            element.plot_elements(plotter=plotter)