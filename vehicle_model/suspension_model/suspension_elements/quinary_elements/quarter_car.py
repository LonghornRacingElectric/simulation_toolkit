from vehicle_model.suspension_model.suspension_elements.quaternary_elements.push_pull_rod import PushPullRod
from vehicle_model.suspension_model.suspension_elements.secondary_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements.secondary_elements.tire import Tire
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.assets.misc_math import unit_vec, rotation_matrix

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
    tire : Tire
        Tire object

    lower_wishbone : Wishbone
        Lower wishbone object

    upper_wishbone : Wishbone
        Upper wishbone object

    tie_rod : Link
        Link object representing tie rod
        
    push_pull_rod : PushPullRod
        PushPullRod object representing push or pull rod
    """
    def __init__(
            self,
            tire: Tire,
            lower_wishbone: Wishbone,
            upper_wishbone: Wishbone,
            tie_rod: Link,
            push_pull_rod: PushPullRod):
        
        self.tire = tire
        self.lower_wishbone = lower_wishbone
        self.upper_wishbone = upper_wishbone
        self.tie_rod = tie_rod
        self.push_pull_rod = push_pull_rod

        # Define fixed geometry with Links
        self.LCA_to_UCA = Link(inboard_node=lower_wishbone.fore_link.outboard_node, outboard_node=upper_wishbone.fore_link.outboard_node)
        self.LCA_to_tire = Link(inboard_node=lower_wishbone.fore_link.outboard_node, outboard_node=self.tire.contact_patch)

        # Save relative coordinates for fixed geometry (relative to outboard pickup on lower wishbone)
        self.tie_rod_wrt_LCA = self.LCA_to_UCA.link_centered_coords(node=self.tie_rod.outboard_node)
        self.tire_wrt_LCA = self.LCA_to_UCA.link_centered_coords(node=self.tire.contact_patch)

        # Save jounce and rack conditions
        self.wheel_jounce: float = 0
        self.rack_displacement: float = 0
    
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
        self.wheel_jounce = jounce
        self._update_geometry()
    
    def steer(self, rack_displacement: float) -> None:
        """
        ## Steer

        Steers quarter car

        Parameters
        ----------
        steer : float
            Lateral rack translation
        
        Returns
        -------
        None
        """
        self.rack_displacement = rack_displacement
        self._update_geometry()
    
    def _update_geometry(self) -> None:
        # Update rack here so it's only updated once
        self.tie_rod.inboard_node.position[1] = self.rack_displacement
        upper_rot, lower_rot, _ = fsolve(func=self._geometry_resid_func, x0=[0, 0, 0])
        
        self.upper_wishbone.rotate(angle=upper_rot)
        self.lower_wishbone.rotate(angle=lower_rot)

    def _geometry_resid_func(self, x: Sequence[float]) -> Sequence[float]:
        """
        ## Geometry Residual Function

        Residual function for goemetry convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution Guess, in the form: [lower_wishbone_rot, upper_wishbone_rot, wheel_angle]

        Returns
        -------
        Sequence[float]
            Residuals
        """
        upper_wishbone_rot = x[0]
        lower_wishbone_rot = x[1]
        wheel_angle = x[2]

        # Apply wishbone rotations. The Node.rotate() method updates the entire system of links, so we'll do this manually.
        # Doing this is about four times quicker.
        upper_rot = rotation_matrix(unit_vec=self.upper_wishbone.direction, theta=upper_wishbone_rot)
        lower_rot = rotation_matrix(unit_vec=self.lower_wishbone.direction, theta=lower_wishbone_rot)
        
        upper_node = self.upper_wishbone.fore_link.outboard_node
        upper_ref = self.upper_wishbone.fore_link.inboard_node
        lower_node = self.lower_wishbone.fore_link.outboard_node
        lower_ref = self.lower_wishbone.fore_link.inboard_node

        upper_node.position = [float(x) for x in np.matmul(upper_rot, np.array(upper_node.initial_position) - np.array(upper_ref.initial_position)) \
                               + np.array(upper_ref.initial_position)]
        lower_node.position = [float(x) for x in np.matmul(lower_rot, np.array(lower_node.initial_position) - np.array(lower_ref.initial_position)) \
                               + np.array(lower_ref.initial_position)]

        # Rotation angles
        ang_x, ang_y = self.LCA_to_UCA.rotation_angles
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)

        # Calculate contact patch location under jounce condition
        self.tire.contact_patch.position = np.matmul(x_rot, np.matmul(y_rot, self.tire_wrt_LCA)) + self.lower_wishbone.fore_link.outboard_node.position

        # Update tie rod pickup consistent with upright (length is NOT preserved)
        self.tie_rod.outboard_node.position = np.matmul(x_rot, np.matmul(y_rot, self.tie_rod_wrt_LCA)) + self.lower_wishbone.fore_link.outboard_node.position

        # Rotate contact_patch and outboard tie_rod pickup position
        self.tire.contact_patch.rotate(origin=self.lower_wishbone.fore_link.outboard_node,
                                       persistent=True,
                                       direction=self.LCA_to_UCA.direction,
                                       angle=wheel_angle)
        
        self.tie_rod.outboard_node.rotate(origin=self.lower_wishbone.fore_link.outboard_node,
                                          persistent=True,
                                          direction=self.LCA_to_UCA.direction,
                                          angle=wheel_angle)

        # Save steered angle
        self.tire.steered_angle = wheel_angle

        # Geometry constraints (kingpin, tie_rod, contact patch)
        kingpin_residual = self.LCA_to_UCA.length - self.LCA_to_UCA.initial_length
        tie_rod_residual = self.tie_rod.length - self.tie_rod.initial_length
        jounce_residual = self.tire.contact_patch[2] - self.wheel_jounce

        return [kingpin_residual, tie_rod_residual, jounce_residual]