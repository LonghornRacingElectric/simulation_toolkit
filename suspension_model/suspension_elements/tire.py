from suspension_model.assets.misc_linalg import rotation_matrix
from suspension_model.suspension_elements.kingpin import Kingpin
from suspension_model.suspension_elements.node import Node
import numpy as np


class Tire:

    def __init__(self, 
                 contact_patch: Node, 
                 kingpin: Kingpin,  
                 static_gamma: float,
                 static_toe: float,
                 radius: float, 
                 width: float) -> None:
        
        self.cp: Node = contact_patch
        self.kingpin = kingpin
        self.gamma: float = static_gamma
        self.radius = radius
        self.width = width
        self.static_toe = static_toe
        self._induced_steer = static_toe

        # Calculate initial center
        rotation = rotation_matrix([1, 0, 0], self.gamma)
        self.initial_center = np.matmul(rotation, [0, 0, 1]) * self.radius + self.cp.position

        # Calculate center relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        center_shifted = self.initial_center - self.kingpin.inboard_node.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.center_to_kingpin = np.matmul(y_rot, np.matmul(x_rot, center_shifted))

        # Calculate contact patch relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        cp_shifted = self.cp.initial_position - self.kingpin.inboard_node.position
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.cp_to_kingpin = np.matmul(y_rot, np.matmul(x_rot, cp_shifted))

        # Calculate static direction
        rotation = rotation_matrix([1, 0, 0], self.gamma)
        self.initial_kpi, self.initial_caster = self.kingpin.normalized_transform()
        self.initial_direction = np.matmul(rotation, [0, 1, 0])
        
        # Calculate direction relative to kingpin
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=-1 * ang_y)
        self.tire_direction = np.matmul(y_rot, np.matmul(x_rot, self.initial_direction))

        self.elements = [self.cp]
    
    def plot_elements(self, plotter):
        plotter.add_tire(center=self.center, direction=self.direction, radius=self.radius, height=self.height)
        for element in self.elements:
            element.plot_elements(plotter=plotter)
    
    @property
    def direction(self):
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.induced_steer)
        direction = np.matmul(y_rot, np.matmul(x_rot, np.matmul(z_rot, self.tire_direction)))

        return direction

    @property
    def center(self):
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self.induced_steer)
        center_position = np.matmul(y_rot, np.matmul(x_rot, np.matmul(z_rot, self.center_to_kingpin))) + self.kingpin.inboard_node.position

        return center_position

    @property
    def height(self):
        return self.width
    
    @property
    def induced_steer(self):
        return self._induced_steer
    
    @induced_steer.setter
    def induced_steer(self, value: float):
        self._induced_steer = self.static_toe
        self._induced_steer += value

        # Update contact patch position
        ang_x, ang_y = self.kingpin.normalized_transform()
        x_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * ang_x)
        y_rot = rotation_matrix(unit_vec=[0, 1, 0], theta=ang_y)
        z_rot = rotation_matrix(unit_vec=[0, 0, 1], theta=self._induced_steer)
        self.cp.position = np.matmul(y_rot, np.matmul(x_rot, np.matmul(z_rot, self.cp_to_kingpin))) + self.kingpin.inboard_node.position