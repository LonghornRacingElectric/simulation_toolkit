from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.suspension_model.assets.misc_linalg import unit_vec
from typing import Sequence
import pyvista as pv
import numpy as np


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
    """
    def __init__(self, inboard: Node, outboard: Node) -> None:
        self.inboard_node: Node = inboard
        self.outboard_node: Node = outboard

        self.elements = [self.inboard_node, self.outboard_node]
        self.all_elements = [self.inboard_node, self.outboard_node]
        self.plotted = False

    def normalized_transform(self) -> Sequence[float]:
        """
        ## Normalized Transform

        Calculates the rotations about x and y which result in a vector pointing strictly in z
        - For the kingpin Link object, this gives kpi and caster, respectively

        Returns
        -------
        Sequence[float]
            Array of rotations in radians [x_rotation, y_rotation]
        """
        origin_transform = self.outboard_node.position - self.inboard_node.position
        ang_x = np.arctan(origin_transform[1] / origin_transform[2])
        ang_y = np.arctan(origin_transform[0] / origin_transform[2])

        return [ang_x, ang_y]

    def plot_elements(self, plotter: pv.Plotter) -> None:
        """
        ## Plot Elements

        Plots all child elements (inboard and outboard Nodes)

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        if max(np.abs(self.inboard_node.position)) < 100:
            plotter.add_link(center=self.center, direction=self.direction, radius=self.radius, height=self.height)
        else:
            plotter.add_link(center=self.center, direction=self.direction, radius=self.radius, height=10, color="blue")

        for element in self.elements:
            element.plot_elements(plotter=plotter)
    
    def yz_intersection(self, link: "Link") -> np.ndarray:
        """
        ## Y-Z Intersection

        Calculates the intersection point between two links in the y-z plane

        Parameters
        ----------
        link : Link
            Second luinkage which intersects self in y-z

        Returns
        -------
        np.ndarray
            Coordinates of intersection
            - Averages x between the two links
        """
        l_1i = self.inboard_node.position
        l_1o = self.outboard_node.position
        m_1 = (l_1o[2] - l_1i[2]) / (l_1o[1] - l_1i[1])
        y_1, z_1 = l_1o[1], l_1o[2]

        l_2i = link.inboard_node.position
        l_2o = link.outboard_node.position
        m_2 = (l_2o[2] - l_2i[2]) / (l_2o[1] - l_2i[1])
        y_2, z_2 = l_2o[1], l_2o[2]

        a = np.array([
            [-1 * m_1, 1],
            [-1 * m_2, 1]
        ])

        b = np.array([
            [-1 * m_1 * y_1 + z_1],
            [-1 * m_2 * y_2 + z_2]
        ])

        y, z = np.linalg.solve(a=a, b=b)

        # Calculate x-value
        # I'll average between left and right halves for KinRC
        x = np.average([l_1o[0], l_2o[0]])

        return np.array([x, y[0], z[0]])

    def xz_intersection(self, link: "Link") -> np.ndarray:
        """
        ## X-Z Intersection

        Calculates the intersection point between two links in the x-z plane

        Parameters
        ----------
        link : Link
            Second luinkage which intersects self in x-z

        Returns
        -------
        np.ndarray
            Coordinates of intersection
            - Averages y between the two links
        """
        l_1i = self.inboard_node.position
        l_1o = self.outboard_node.position
        m_1 = (l_1o[2] - l_1i[2]) / (l_1o[0] - l_1i[0])
        x_1, z_1 = l_1o[0], l_1o[2]

        l_2i = link.inboard_node.position
        l_2o = link.outboard_node.position
        m_2 = (l_2o[2] - l_2i[2]) / (l_2o[0] - l_2i[0])
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
        # I'll average between front and rear halves for KinRC
        y = np.average([l_1o[1], l_2o[1]])

        try:
            x, z = np.linalg.solve(a=a, b=b)
        except:
            y = np.average([l_1o[1], l_2o[1]])
            return [1e9, y, 0]

        return np.array([x[0], y, z[0]])

    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children (inboard and outboard Nodes)

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

        Rotates all children (inboard and outboard nodes)
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        angle : Sequence[float]
            Angles of rotation in radians [x_rot, y_rot, z_rot]
        """
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)
    
    @property
    def direction(self) -> np.ndarray:
        """
        ## Direction

        Direction attribute of Link

        Returns
        -------
        np.ndarray
            Direction of Link
        """
        return unit_vec(self.outboard_node, self.inboard_node)

    @property
    def center(self) -> np.ndarray:
        """
        ## Center
        
        Center attribute of link

        Returns
        -------
        np.ndarray
            Center of link
        """
        if 1e9 not in np.abs(self.inboard_node.position):
            return (self.inboard_node.position + self.outboard_node.position) / 2
        else:
            return (self.outboard_node.position)
    
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
    def height(self) -> float:
        """
        ## Height

        Height (length) attribute of link

        Returns
        -------
        float
            Length of link
        """
        return float(np.linalg.norm(self.inboard_node.position - self.outboard_node.position))