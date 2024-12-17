from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.assets.misc_linalg import unit_vec
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
    """
    def __init__(self, inboard_node: Node, outboard_node: Node) -> None:
        
        self.inboard_node: Node = inboard_node
        self.outboard_node: Node = outboard_node
    
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

        try:
            y, z = np.linalg.solve(a=a, b=b).flatten()
        except np.linalg.LinAlgError:
            warnings.warn("\nSingular Matrix Encountered | yz intersection assumed at infinity. This is not a critical error, but check results carefully.")
            y, z = np.inf, np.average([z_2, z_1])

        # Calculate x-value
        # I'll average between left and right halves for KinRC
        x = np.average([l_1o[0], l_2o[0]])

        return Node(position=[x, y, z])

    def xz_intersection(self, link: "Link") -> Node:
        """
        ## x-z Intersection

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
        # I'll average between front and rear halves for KinPC
        y = np.average([l_1o[1], l_2o[1]])

        try:
            x, z = np.linalg.solve(a=a, b=b).flatten()
        except:
            warnings.warn("\nSingular Matrix Encountered | xz intersection assumed at infinity. This is not a critical error, but check results carefully.")
            x, z = np.inf, np.average([z_2, z_1])

        coords = [float(x) for x in [x, y, z]]

        return Node(position=coords)
    
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
        return unit_vec(p1=self.inboard_node.position, p2=self.outboard_node.position)

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
        return (self.inboard_node.position + self.outboard_node.position) / 2
    
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