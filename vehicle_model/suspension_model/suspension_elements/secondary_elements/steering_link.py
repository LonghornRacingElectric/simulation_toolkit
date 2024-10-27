from vehicle_model.suspension_model.suspension_elements.secondary_elements.kingpin import Kingpin
from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.assets.misc_linalg import rotation_matrix
import numpy as np


class SteeringLink(Link):
    """
    ## Steering Link

    Steering Link object
    - Similar to beam, defined by two nodes

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of steering linkage
    outboard : Node
        Node representing outboard end of steering linkage
    """
    def __init__(self, inboard: Node, outboard: Node, kingpin: Kingpin) -> None:
        super().__init__(inboard=inboard, outboard=outboard)

        self.kingpin = kingpin
        self.initial_length = self.length
        self.steering_pickup_to_kingpin = self._steering_pickup_to_kingpin
        self.angle = 0

    @property
    def length(self) -> float:
        """
        ## Length

        Length of steering linkage

        Returns
        -------
        float
            Length of steering linkage
        """
        return np.linalg.norm(self.inboard_node.position - self.outboard_node.position)

    @property
    def _steering_pickup_to_kingpin(self) -> Sequence[float]:
        """
        ## Steering Pikcup to Kingpin

        Returns
        -------
        float
            Position vector from steering pickup to the lower kingpin node
        """
        ang_x, ang_y = self.kingpin.normalized_transform()
        steering_pickup_pos_shifted = self.outboard_node.position - self.kingpin.inboard_node.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        
        return np.matmul(y_rot, np.matmul(x_rot, steering_pickup_pos_shifted))

    def update(self) -> None:
        """
        ## Update

        Updates steering pickup position based on jounce condition

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        # Recalculate steering pickup based on jounce condition
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.angle)
        steering_pickup_position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.steering_pickup_to_kingpin))) + self.kingpin.inboard_node.position

        self.outboard_node.position = steering_pickup_position
    
    def rotate(self, angle: float) -> None:
        """
        ## Rotate

        Rotates steering pickup about kingpin

        Parameters
        ----------
        angle : float
            Angle of rotation in radians
        
        Returns
        -------
        None
        """
        self._set_initial_position()
        self.angle = angle
        self.update()
    
    def _set_initial_position(self) -> None:
        """
        ## Set Initial Position

        Resets position of steering link to initial position

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        kingpin_rot = rotation_matrix(unit_vec=self.kingpin.direction, theta=-1 * self.angle)
        steer_about_origin = self.outboard_node.position - self.kingpin.inboard_node.position
        self.outboard_node.position = np.matmul(kingpin_rot, steer_about_origin) + self.kingpin.inboard_node.position