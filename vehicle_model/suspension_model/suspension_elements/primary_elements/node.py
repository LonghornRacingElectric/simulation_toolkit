from vehicle_model.assets.updateable import Updateable

from vehicle_model.assets.misc_linalg import unit_vec, rotation_matrix

from typing import Sequence, Union, List, Any
from copy import deepcopy
import numpy as np


class Node:
    """
    ## Node

    Node object
    - Similar to node element

    Parameters
    ----------
    Position : Sequence[float]
        Position of Node
    """
    def __init__(self, position: Union[np.ndarray, Sequence[float]]) -> None:
        self.listeners: Sequence[Updateable] = []
        self.child_nodes: List[Node] = []

        self.position: list = deepcopy(list(position))
        self.initial_position: list = deepcopy(list(position))

    def reset(self) -> None:
        """
        ## Reset

        Resets Node position to initial position

        Parameters
        ----------
        None

        Returns
        ----------
        None
        """
        self.position = self.initial_position
        self.__update_children__()
    
    def translate(self, translation: Union[np.ndarray, Sequence[float]]) -> None:
        """
        ## Translate

        Translates Node

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply

        Returns
        ----------
        None
        """
        self.position = [x + y for x, y in zip(self.position, translation)]
        self.__update_children__()
    
    def rotate(self, origin: "Node", angle_x: float = 0, angle_y: float = 0, angle_z: float = 0) -> None:
        """
        ## Flatten Rotate

        Rotates Node
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        origin : Node
            Node representing origin of transformation
        angle_x : float
            Angle of rotation about x in radians
        angle_y : float
            Angle of rotation about y in radians
        angle_z : float
            Angle of rotation about z in radians
        """
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=angle_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=angle_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=angle_z)
        
        rotation_wrt_origin = np.matmul(z_rot, np.matmul(y_rot, np.matmul(x_rot, (self - origin).position)))
        self.position =  [x + y for x, y in zip(rotation_wrt_origin, origin.position)]
        self.__update_children__()
    
    def add_child(self, node: "Node") -> None:
        """
        ## Add Child

        Adds child node to self

        Parameters
        ----------
        node : Node
            Child node to add to self
        """
        self.child_nodes.append(node)
    
    def __add__(self, node: "Node") -> "Node":
        """
        ## Overloaded Node Addition

        Allows for direct addition of nodes

        Parameters
        ----------
        node : Node
            Node to add to desired Node

        Returns
        -------
        Node
            Node with position equal to the sum of self.position and node.position
        """
        node_sum = [self.position[0] + node.position[0],
                    self.position[1] + node.position[1],
                    self.position[2] + node.position[2]]

        return Node(position=node_sum)

    def __sub__(self, node: "Node") -> "Node":
        """
        ## Overloaded Node Subtraction

        Allows for direct subtraction of nodes

        Parameters
        ----------
        node : Node
            Node to subtract from desired Node

        Returns
        -------
        Node
            Node with position equal to the difference of self.position and node.position
        """
        node_sub = [self.position[0] - node.position[0],
                    self.position[1] - node.position[1],
                    self.position[2] - node.position[2]]

        return Node(position=node_sub)
    
    def __mul__(self, num: Union[float, int]):
        """
        ## Overloaded Node Multiplication

        Multiplies all entries by the specified value

        Parameters
        ----------
        num : Union[float, int]
            Number to multiply all entries by
        """
        node_mul = [x * num for x in self.position]

        return Node(position=node_mul)

    def __truediv__(self, num: Union[float, int]):
        """
        ## Overloaded Node Division

        Divides all entries by the specified value

        Parameters
        ----------
        num : Union[float, int]
            Number to divide all entries by
        """
        node_div = [x / num for x in self.position]

        return Node(position=node_div)
    
    def __getitem__(self, index: int) -> float:
        """
        ## Node Indexing

        Allows for node indexing, based on position array

        Parameters
        ----------
        index : int
            Position entry to extract

        Returns
        -------
        float
            Value corresponding to entry index
        """
        value = self.position[index]

        return value
    
    def __setitem__(self, index: int, value: Union[float, int]) -> None:
        """
        ## Node Index Setting

        Allows for setting node values, based on indexed position array

        Parameters
        ----------
        index : int
            Position entry to set

        value : Union[float, int]
            Value to set at position entry
        """
        self.position[index] = value
        self.__update_children__()
    
    def __update_children__(self) -> None:
        """
        ## Update Children

        Updates child Nodes

        Parameters
        ----------
        None
        """
        for node in self.child_nodes:
            trans_vec = np.array(self.position) - np.array(self.initial_position)
            node.position = list(np.array(node.initial_position) + trans_vec)

        for listener in self.listeners:
            listener.update()