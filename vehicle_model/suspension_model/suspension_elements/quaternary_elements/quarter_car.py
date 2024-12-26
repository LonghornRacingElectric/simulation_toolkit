from vehicle_model.suspension_model.suspension_elements.secondary_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.assets.misc_linalg import unit_vec, rotation_matrix
# from scipy.interpolate import CubicSpline
from typing import Sequence, Tuple, Union
from scipy.optimize import fsolve # type: ignore
import numpy as np


class QuarterCar:
    """
    ## Quarter Car

    Full corner assembly
    - Includes wishbones, tie rod, and push/pull rod

    Parameters
    ----------
    lower_wishbone : Wishbone
        Lower wishbone object

    upper_wishbone : Wishbone
        Upper wishbone object

    tie_rod : Link
        Link object representing tie rod
        
    push_pull_rod : Link
        Link object representing push or pull rod
    """
    def __init__(
            self,
            lower_wishbone: Wishbone,
            upper_wishbone: Wishbone,
            tie_rod: Link,
            push_pull_rod: Link):

        self.lower_wishbone = lower_wishbone
        self.upper_wishbone = upper_wishbone
        self.tie_rod = tie_rod
        self.push_pull_rod = push_pull_rod

        # Define kingpin from wishbones
        self.kingpin = Link(inboard_node=lower_wishbone.fore_link.outboard_node, outboard_node=upper_wishbone.fore_link.outboard_node)
        
        # Define fixed geometry to tie rod
        self.tie_rod_unsprung = self.kingpin.link_centered_coords(node=self.tie_rod.outboard_node)

        # Save steering contributions
        self.rack_steer: Union[float, int] = 0
        self.jounce_steer_contribution: Union[float, int] = 0
        self.driver_steer_contribution: Union[float, int] = 0

        self.wheel_angle: Union[float, int] = self.jounce_steer_contribution + self.driver_steer_contribution

    def jounce(self, jounce: float) -> None:
        """
        ## Jounce

        Updates double wishbone geometry for given vertical travel of the contact patch (jounce)

        Parameters
        ----------
        jounce : float, optional
            Vertical travel of the contact patch in meters
                
        Returns
        -------
        None
        """
        # Rotate wishbones
        fsolve(func=self._jounce_eqn, x0=[0, 0], args=[jounce])

        # Reset outboard tie rod pickup to match unsprung geometry
        self.reset_jounce_steer()

        # Rotate wheel
        self.jounce_steer_contribution = fsolve(func=self._steer_jounce_eqn, x0=[0], args=[self.kingpin.direction])[0]  

        # Update driver steer contribution
        self.steer(rack_translation=self.rack_steer)

        # Update total steer
        self.wheel_angle = self.jounce_steer_contribution + self.driver_steer_contribution
    
    def steer(self, rack_translation: float) -> None:
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
        # Save rack translation for persistent states
        self.rack_steer = rack_translation

        # Adjust rack
        self.tie_rod.inboard_node.position[1] = self.tie_rod.inboard_node.initial_position[1] + rack_translation

        # Reset outboard tie rod pickup to match unsprung geometry
        self.reset_driver_steer()

        # Rotate wheel
        self.driver_steer_contribution = fsolve(func=self._steer_eqn, x0=[0], args=[self.kingpin.direction])[0]

        # Update total steer
        self.wheel_angle = self.jounce_steer_contribution + self.driver_steer_contribution
    
    def reset_jounce_steer(self) -> None:
        # Reset outboard tie rod pickup to match unsprung geometry, considering driver steering contribution
        ang_x, ang_y = self.kingpin.rotation_angles
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.driver_steer_contribution)
        self.tie_rod.outboard_node.position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.tie_rod_unsprung))) + self.kingpin.inboard_node.position
    
    def reset_driver_steer(self) -> None:
        # Reset outboard tie rod pickup to match unsprung geometry, considering jounce steering contribution
        ang_x, ang_y = self.kingpin.rotation_angles
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.jounce_steer_contribution)
        self.tie_rod.outboard_node.position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.tie_rod_unsprung))) + self.kingpin.inboard_node.position

    def _jounce_eqn(self, x: Sequence[float], args: Sequence[float]) -> Tuple[float, float]:
        theta_L = x[0]
        theta_U = x[1]

        dz = args[0]

        self.lower_wishbone.rotate(angle=theta_L)
        self.upper_wishbone.rotate(angle=theta_U)

        lower_outboard = self.lower_wishbone.fore_link.outboard_node
        upper_outboard = self.upper_wishbone.fore_link.outboard_node

        # Distance constraint
        original_dist = np.linalg.norm(np.array(lower_outboard.initial_position) - np.array(upper_outboard.initial_position)).__float__()
        new_dist = np.linalg.norm((lower_outboard - upper_outboard).position).__float__()

        # Jounce constraint
        new_z = lower_outboard[2].__float__()

        return (original_dist - new_dist, new_z - dz)

    def _steer_jounce_eqn(self, x: Sequence[float], args: Sequence[np.ndarray]) -> Sequence[float]:
        delta = x[0]
        direc = args[0]

        # Reset outboard tie rod pickup to match unsprung geometry
        self.reset_jounce_steer()

        # Define nodes
        outboard_tie = self.tie_rod.outboard_node
        inboard_tie = self.tie_rod.inboard_node

        # Apply rotation
        outboard_tie.position = np.matmul(rotation_matrix(unit_vec=direc, theta=delta), \
                                          (outboard_tie - self.kingpin.inboard_node).position) + self.kingpin.inboard_node.position

        original_dist = np.linalg.norm(np.array(outboard_tie.initial_position) - np.array(inboard_tie.initial_position)).__float__()
        new_dist = np.linalg.norm((outboard_tie - inboard_tie).position).__float__()

        return [original_dist - new_dist]
    
    def _steer_eqn(self, x: Sequence[float], args: Sequence[np.ndarray]) -> Sequence[float]:
        delta = x[0]
        direc = args[0]

        # Reset outboard tie rod pickup to match unsprung geometry
        self.reset_driver_steer()

        # Define nodes
        outboard_tie = self.tie_rod.outboard_node
        inboard_tie = self.tie_rod.inboard_node

        # Apply rotation
        rot = rotation_matrix(unit_vec=direc, theta=delta)
        outboard_tie.position = np.matmul(rot, (outboard_tie - self.kingpin.inboard_node).position) + self.kingpin.inboard_node.position

        original_dist = np.linalg.norm(np.array(outboard_tie.initial_position) - np.array(inboard_tie.initial_position)).__float__()
        new_dist = np.linalg.norm((outboard_tie - inboard_tie).position).__float__()

        return [original_dist - new_dist]

    # @property
    # def motion_ratio(self) -> float:
    #     """
    #     ## Motion Ratio

    #     Motion Ratio attribute under current jounce condition

    #     Returns
    #     -------
    #     float
    #         Motion ratio under current jounce condition
    #     """
    #     # return self.motion_ratio_function(self.total_jounce)

    # @property
    # def wheelrate(self) -> float:
    #     """
    #     ## Wheelrate

    #     Calculates wheelrate under current jounce condition

    #     Returns
    #     -------
    #     float
    #         Wheelrate under current jounce condition
    #     """
    #     pass
        # return self.spring_rate / 1.4**2
    
    # @property
    # def FVIC_position(self) -> Sequence[float]:
    #     """
    #     ## FVIC Position

    #     Calculates position of front-view instance center

    #     Returns
    #     -------
    #     Sequence[float]
    #         Coordinates of front-view instance center
    #     """
    #     upper_plane = self.upper_wishbone.plane
    #     lower_plane = self.lower_wishbone.plane
    #     x = self.contact_patch.position[0]

    #     if (upper_plane[1] / upper_plane[2]) == (lower_plane[1] / lower_plane[2]):
    #         return [x, -1 * 1e9 * np.sign(self.contact_patch.position[1]), 0]

    #     a = np.array(
    #         [
    #             [upper_plane[1], upper_plane[2]],
    #             [lower_plane[1], lower_plane[2]]
    #         ])
        
    #     b = np.array(
    #         [
    #             [upper_plane[0] * (upper_plane[3] - x) + upper_plane[1] * upper_plane[4] + upper_plane[2] * upper_plane[5]],
    #             [lower_plane[0] * (lower_plane[3] - x) + lower_plane[1] * lower_plane[4] + lower_plane[2] * lower_plane[5]]
    #         ])

    #     soln = np.linalg.solve(a=a, b=b)

    #     y = soln[0][0]
    #     z = soln[1][0]

    #     return [x, y, z]
    
    # @property
    # def SVIC_position(self) -> Sequence[float]:
    #     """
    #     ## SVIC Position

    #     Calculates position of side-view instance center

    #     Returns
    #     -------
    #     Sequence[float]
    #         Coordinates of side-view instance center
    #     """
    #     upper_plane = self.upper_wishbone.plane
    #     lower_plane = self.lower_wishbone.plane
    #     y = self.contact_patch.position[1]

    #     if (upper_plane[0] / upper_plane[2]) == (lower_plane[0] / lower_plane[2]):
    #         if self.contact_patch.position[0] == 0:
    #             return [-1 * 1e9, y, 0]
    #         else:
    #             return [-1 * 1e9 * np.sign(self.contact_patch.position[0]), y, 0]

    #     a = np.array(
    #         [
    #             [upper_plane[0], upper_plane[2]],
    #             [lower_plane[0], lower_plane[2]]
    #         ])
        
    #     b = np.array(
    #         [
    #             [upper_plane[1] * (upper_plane[4] - y) + upper_plane[0] * upper_plane[3] + upper_plane[2] * upper_plane[5]],
    #             [lower_plane[1] * (lower_plane[4] - y) + lower_plane[0] * lower_plane[3] + lower_plane[2] * lower_plane[5]]
    #         ])

    #     soln = np.linalg.solve(a=a, b=b)

    #     x = soln[0][0]
    #     z = soln[1][0]

    #     return [x, y, z]

    # @property
    # def FV_FAP_position(self) -> Sequence[float]:
    #     """
    #     ## Front-view force application point height

    #     Calculates the position of the front-view force application point

    #     Returns
    #     -------
    #         Height of the front-view force application point
    #     """

    #     dir_yz = self.FVIC_link.inboard_node.position - self.FVIC_link.outboard_node.position
    #     z = (dir_yz[2] / dir_yz[1]) * (self.cg.position[1] - self.FVIC_link.outboard_node.position[1]) + self.FVIC_link.outboard_node.position[2]

    #     x = self.FVIC_link.outboard_node.position[0]
    #     y = self.cg.position[1]

    #     return [x, y, z]
    
    # @property
    # def SV_FAP_position(self) -> float:
    #     """
    #     ## Side-view force application point height

    #     Calculates the height of the side-view force application point

    #     Returns
    #     -------
    #         Height of the side-view force application point
    #     """

    #     dir_xz = self.SVIC_link.inboard_node.position - self.SVIC_link.outboard_node.position
    #     z = (dir_xz[2] / dir_xz[0]) * (self.cg.position[0] - self.SVIC_link.outboard_node.position[0]) + self.SVIC_link.outboard_node.position[2]

    #     x = self.cg.position[0]
    #     y = self.SVIC_link.outboard_node.position[1]
        
    #     return [x, y, z]

    # @property
    # def caster(self) -> float:
    #     """
    #     ## Caster

    #     Calculates caster

    #     Returns
    #     -------
    #     float
    #         Caster angle in radians
    #     """
    #     return self.kingpin.normalized_transform()[1]

    # @property
    # def kpi(self) -> float:
    #     """
    #     ## KPI

    #     Calculates kingpin inclination

    #     Returns
    #     -------
    #     float
    #         Kingpin inclination angle in radians
    #     """
    #     return self.kingpin.normalized_transform()[0]
    
    # @property
    # def scrub(self) -> float:
    #     kpi_dir = self.kingpin.direction
    #     center = self.kingpin.center
        
    #     x = center[0] - center[2] / kpi_dir[2] * kpi_dir[0]
    #     y = center[1] - center[2] / kpi_dir[2] * kpi_dir[1]

    #     ground_pierce = np.array([x, y, 0])
    #     pierce_to_cp = self.contact_patch.position - ground_pierce

    #     return pierce_to_cp[1]

    # @property
    # def toe(self) -> float:
    #     """
    #     ## Toe

    #     Calculates toe angle

    #     Returns
    #     -------
    #     float
    #         Toe angle in radians
    #     """
    #     return self.tire.induced_steer

    # @property
    # def inclination_angle(self) -> float:
    #     """
    #     ## Inclination Angle

    #     Calculates inclination angle

    #     Returns
    #     -------
    #     float
    #         Inclination angle in radians
    #     """
    #     # This only works because of the way I set up the direction vectors
    #     # You'll likely run into sign issues if applying this elsewhere
    #     vec_a = np.array(self.tire.direction)
    #     gamma = np.arctan(vec_a[2] / (np.sqrt(vec_a[0]**2 + vec_a[1]**2)))

    #     return gamma