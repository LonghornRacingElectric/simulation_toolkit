from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
from _4_custom_libraries.misc_math import unit_vec, rotation_matrix
from typing import Sequence, Union
import numpy as np
import warnings


class Link:
    """
    ## Link

    Link object
    - Similar to beam, defined by two nodes

    Parameters
    ----------
    inboard : Node
        Node representing inboard end of linkage

    outboard : Node
        Node representing outboard end of linkage
        
    **kwargs : dict[str, Union[float, int, str]]
        Optional keyword arguments. The following keys are supported:

            `compliance` : float
                Linear compliance of Link, by default None

            `compliance_unit` : str
                Unit of Link compliance. Options include:
                
                `N/m`
                `N/mm`
                `lbf/in`
                `lb/in`
    """
    def __init__(self, inboard_node: Node, outboard_node: Node, **kwargs) -> None:
        
        self.inboard_node: Node = inboard_node
        self.outboard_node: Node = outboard_node
        self.initial_length: float = np.linalg.norm((self.outboard_node - self.inboard_node).position).__float__()

        self.compliance: Union[None, float, int] = None
        self.compliance_unit: Union[None, str] = None

        if "compliance" in kwargs:
            self.compliance = kwargs.pop("compliance")

        # if ("compliance" in kwargs) ^ ("compliance_units" in kwargs):
        #     raise Exception("Both compliance and compliance_units need to be specified in Link object")
        # elif "compliance" in kwargs:
        #     self.compliance = kwargs.pop("compliance")
        #     self.compliance_unit = kwargs.pop("compliance_unit")

    def yz_intersection(self, link: "Link") -> Node:
        """
        ## y-z Intersection

        Calculates the intersection point between two links in the y-z plane

        Parameters
        ----------
        link : Link
            Second luinkage which intersects self in y-z

        Returns
        -------
        Node
            Node coincident with intersection
            - Averages x between the two links
        """
        l_1i = self.inboard_node
        l_1o = self.outboard_node
        m_1 = (l_1o - l_1i)[2] / (l_1o - l_1i)[1]
        y_1, z_1 = l_1o[1], l_1o[2]

        l_2i = link.inboard_node
        l_2o = link.outboard_node
        m_2 = (l_2o - l_2i)[2] / (l_2o - l_2i)[1]
        y_2, z_2 = l_2o[1], l_2o[2]

        a = np.array([
            [-1 * m_1, 1],
            [-1 * m_2, 1]
        ])

        b = np.array([
            [-1 * m_1 * y_1 + z_1],
            [-1 * m_2 * y_2 + z_2]
        ])

        try:
            y, z = np.linalg.solve(a=a, b=b).flatten()
        except np.linalg.LinAlgError:
            warnings.warn("\nSingular Matrix Encountered | yz intersection assumed at infinity. This is not a critical error, but check results carefully.")
            y, z = np.inf, np.average([z_2, z_1])

        # Calculate x-value
        # I'll average between left and right halves for KinRC
        x = np.average([l_1o[0], l_2o[0]]).__float__()

        return Node(position=[x, y, z])

    def xz_intersection(self, link: "Link") -> Node:
        """
        ## x-z Intersection

        Calculates the intersection point between two links in the x-z plane

        Parameters
        ----------
        link : Link
            Second linkage which intersects self in x-z

        Returns
        -------
        np.ndarray
            Coordinates of intersection
            - Averages y between the two links
        """
        l_1i = self.inboard_node
        l_1o = self.outboard_node
        m_1 = (l_1o - l_1i)[2] / (l_1o - l_1i)[0]
        x_1, z_1 = l_1o[0], l_1o[2]

        l_2i = link.inboard_node
        l_2o = link.outboard_node
        m_2 = (l_2o - l_2i)[2] / (l_2o - l_2i)[0]
        x_2, z_2 = l_2o[0], l_2o[2]

        a = np.array([
            [-1 * m_1, 1],
            [-1 * m_2, 1]
        ])

        b = np.array([
            [-1 * m_1 * x_1 + z_1],
            [-1 * m_2 * x_2 + z_2]
        ])
        
        # Calculate y-value
        # I'll average between front and rear halves for KinPC
        y = np.average([l_1o[1], l_2o[1]])

        try:
            x, z = np.linalg.solve(a=a, b=b).flatten()
        except:
            warnings.warn("\nSingular Matrix Encountered | xz intersection assumed at infinity. This is not a critical error, but check results carefully.")
            x, z = np.inf, np.average([z_2, z_1])

        coords = [float(x) for x in [x, y, z]]

        return Node(position=coords)

    def link_centered_coords(self, node: Node) -> np.ndarray:
        """
        ## Link-Centered Coordinates

        Calculates Node coordinates with Link treated as z-axis

        Parameters
        ----------
        node : Node
            Node to represent in Link reference frame

        Returns
        -------
        Sequence[float]
            Node coordinates in Link reference frame
        """
        ang_x, ang_y = self.rotation_angles
        node_translated = node - self.inboard_node

        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        
        node_coords_rotated: np.ndarray = np.matmul(y_rot, np.matmul(x_rot, node_translated.position))
        return node_coords_rotated

    @property
    def component_angles(self) -> Sequence[float]:
        """
        ## Component Angles

        Calculates the smallest angles between the ground plane and the projection of Link on the principal planes
        - For a kingpin Link, this gives kpi and caster, respectively

        Returns
        -------
        Sequence[float]
            Sequence of angles in radians [ang_x, ang_y]
        """
        origin_transform = self.outboard_node - self.inboard_node
        ang_x = np.arctan(origin_transform[2] / origin_transform[1]).__float__()
        ang_y = np.arctan(origin_transform[2] / origin_transform[0]).__float__()

        return [ang_x, ang_y]
    
    @property
    def rotation_angles(self) -> Sequence[float]:
        """
        ## Rotation Angles

        Calculates the rotations about x and y which result in a vector pointing strictly in z

        Returns
        -------
        Sequence[float]
            Sequence of rotations in radians [x_rotation, y_rotation]
        """
        origin_transform = self.outboard_node - self.inboard_node
        ang_x = np.arctan(origin_transform[1] / origin_transform[2]).__float__()
        ang_y = np.sign(origin_transform[2]) * np.arcsin(origin_transform[0] / self.length).__float__()

        return [ang_x, ang_y]
    
    @property
    def direction(self) -> Sequence[float]:
        """
        ## Direction

        Direction attribute of Link

        Returns
        -------
        np.ndarray
            Direction of Link
        """
        return unit_vec(p1=self.inboard_node.position, p2=self.outboard_node.position)

    @property
    def center(self) -> Sequence[float]:
        """
        ## Center
        
        Center attribute of link

        Returns
        -------
        np.ndarray
            Center of link
        """
        new_node = (self.inboard_node + self.outboard_node) / 2

        return new_node.position
    
    @property
    def radius(self) -> float:
        """
        ## Radius

        Radius attribute of link

        Returns
        -------
        float
            Radius of link
        """
        return 0.015875 / 2

    @property
    def length(self) -> float:
        """
        ## Length

        Length of link

        Returns
        -------
        float
            Length of link
        """
        return np.linalg.norm((self.outboard_node - self.inboard_node).position).__float__()