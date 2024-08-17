import suspension_model.assets.misc_linalg as linalg
from typing import Sequence
import numpy as np


class Node:
    def __init__(self, position: list) -> None:
        self.position = np.array(position)
        self.initial_position = position

    def reset(self):
        self.position = self.initial_position
    
    def translate(self, translation: Sequence[float]):
            self.position = self.position + np.array(translation)
    
    def flatten_rotate(self, angle: Sequence[float]):
        x_rot = linalg.rotation_matrix(unit_vec=[1, 0, 0], theta=angle[0])
        y_rot = linalg.rotation_matrix(unit_vec=[0, 1, 0], theta=angle[1])
        z_rot = linalg.rotation_matrix(unit_vec=[0, 0, 1], theta=angle[2])
        self.position = np.matmul(x_rot, np.matmul(y_rot, np.matmul(z_rot, self.position)))

    def plot_elements(self, plotter, radius: float = 0.875 / 2):
        if max(np.abs(self.position)) < 1000:
            plotter.add_node(center=self.position, radius=radius)