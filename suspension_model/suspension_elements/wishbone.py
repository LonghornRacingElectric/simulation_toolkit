from suspension_model.assets.misc_linalg import rotation_matrix
from suspension_model.assets.misc_linalg import unit_vec
from suspension_model.suspension_elements.link import Link
from suspension_model.assets.plotter import Plotter
import matplotlib.pyplot as plt
import numpy as np


class Wishbone:
    def __init__(self, fore_link: Link, aft_link: Link) -> None:
        self.fore_link = fore_link
        self.aft_link = aft_link

        self.direction = self.direction_vec
        self.angle = 0

        self.elements = [self.fore_link, self.aft_link]

    def plot_elements(self, plotter):
        for element in self.elements:
            element.plot_elements(plotter=plotter)

    def rotate(self, angle: float):
        self._set_initial_position()
        
        self.rot_mat = rotation_matrix(self.direction, angle)
        outboard_point = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position
        self.fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position

        self.angle = angle
    
    def _set_initial_position(self):
        self.rot_mat = rotation_matrix(self.direction, -1 * self.angle)
        outboard_point = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position

        self.fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position

    @property
    def plane(self):
        # General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
        PQ = self.fore_link.outboard_node.position - self.fore_link.inboard_node.position
        PR = self.aft_link.outboard_node.position - self.aft_link.inboard_node.position

        a, b, c = np.cross(PQ, PR)
        x_0, y_0, z_0 = self.fore_link.outboard_node.position

        return [a, b, c, x_0, y_0, z_0]

    @property
    def direction_vec(self):
        return unit_vec(self.fore_link.inboard_node, self.aft_link.inboard_node)