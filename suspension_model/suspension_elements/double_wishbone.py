from suspension_model.suspension_elements.steering_link import SteeringLink
from suspension_model.suspension_elements.wishbone import Wishbone
from suspension_model.assets.misc_linalg import rotation_matrix
from suspension_model.suspension_elements.kingpin import Kingpin
from suspension_model.suspension_elements.link import Link
from suspension_model.suspension_elements.tire import Tire
from suspension_model.suspension_elements.node import Node
from suspension_model.assets.plotter import Plotter
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class DoubleWishbone:
    def __init__(
            self,
            inboard_points: Sequence[Sequence[float]],
            outboard_points: Sequence[Sequence[float]],
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
        
        # Initialize state and plotter
        self.current_jounce = 0
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
        self.tire: Tire = Tire(contact_patch=self.contact_patch, kingpin=self.kingpin, static_gamma=inclination_angle, static_toe=toe, radius=tire_radius, width=tire_width)

        # Define instant centers
        self.FVIC = Node(position=self.FVIC_position)
        self.FVIC_link = Link(inboard=self.FVIC, outboard=self.contact_patch)
        self.SVIC = Node(position=self.SVIC_position)
        self.SVIC_link = Link(inboard=self.SVIC, outboard=self.contact_patch)

        # Cache unsprung geometry
        self._fixed_unsprung_geom()

        if not show_ICs:
            self.elements = [self.upper_wishbone, self.lower_wishbone, self.kingpin, self.steering_link, self.tire]
        else:
            self.elements = [self.kingpin, self.steering_link, self.tire, self.FVIC, self.SVIC, self.FVIC_link, self.SVIC_link]

    def _fixed_unsprung_geom(self):
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

    def _jounce_resid_func(self, x: Sequence[float], jounce: float):
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

    def _jounce_induced_steer_resid_func(self, x):
        induced_steer = x[0]

        # Apply induced steering rotation
        self.steering_link.rotate(induced_steer)

        residual_length = self.steering_link.length - self.steering_link.initial_length

        return residual_length

    def jounce(self, jounce: float):
        wishbone_angles = fsolve(self._jounce_resid_func, [0, 0], args=(jounce))

        self.upper_wishbone.rotate(wishbone_angles[0])
        self.lower_wishbone.rotate(wishbone_angles[1])

        induced_steer = fsolve(self._jounce_induced_steer_resid_func, [0])
        self.steering_link.rotate(induced_steer[0])
        
        # Set jounce-induced steer in tire
        self.tire.induced_steer = induced_steer[0]

        self.current_jounce = jounce
        self.induced_steer = induced_steer[0]
        self.FVIC.position = self.FVIC_position
        self.SVIC.position = self.SVIC_position

    def _steer_resid_func(self, x):
        steer_angle = x[0]

        self.steering_link.rotate(steer_angle)
        residual_length = self.steering_link.length - self.steering_link.initial_length

        return [residual_length]
    
    def steer(self, steer: float):
        self.steering_link.inboard_node.position[1] = self.steering_link.inboard_node.initial_position[1] + steer

        angle = fsolve(self._steer_resid_func, [0])

        self.tire.induced_steer = angle[0] + self.induced_steer

        self.steered_angle = steer
    
    @property
    def FVIC_position(self):
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
    def SVIC_position(self):
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
    def caster(self):
        return self.kingpin.normalized_transform()[1]

    @property
    def kpi(self):
        return self.kingpin.normalized_transform()[0]

    @property
    def toe(self):
        return self.tire.induced_steer * 180 / np.pi

    @property
    def inclination_angle(self):
        vec_a = np.array(self.tire.direction)

        gamma = (np.arccos(vec_a) * np.sign(self.tire.direction[1] * self.tire.direction[2]))[2] + np.pi / 2
        if gamma > np.pi / 2:
            gamma = abs(gamma - np.pi)

        return gamma
    
    def plot_elements(self, plotter):
        for element in self.elements:
            element.plot_elements(plotter=plotter)