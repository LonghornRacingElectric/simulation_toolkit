from suspension_model.suspension_elements.node import Node
from suspension_model.assets.misc_linalg import unit_vec
from typing import Sequence
import numpy as np


class Link:

    def __init__(self, inboard: Node, outboard: Node) -> None:
        self.inboard_node: Node = inboard
        self.outboard_node: Node = outboard

        self.elements = [self.inboard_node, self.outboard_node]
        self.plotted = False

    def normalized_transform(self) -> Sequence[float]:
        origin_transform = self.outboard_node.position - self.inboard_node.position
        ang_x = np.arctan(origin_transform[1] / origin_transform[2])
        ang_y = np.arctan(origin_transform[0] / origin_transform[2])

        return [ang_x, ang_y]

    def plot_elements(self, plotter):
        if 1e9 not in np.abs(self.inboard_node.position):
            plotter.add_link(center=self.center, direction=self.direction, radius=self.radius, height=self.height)
        else:
            plotter.add_link(center=self.center, direction=self.direction, radius=self.radius, height=10, color="blue")

        for element in self.elements:
            element.plot_elements(plotter=plotter)
    
    @property
    def direction(self) -> np.ndarray:
        return unit_vec(self.outboard_node, self.inboard_node)

    @property
    def center(self) -> np.ndarray:
        if 1e9 not in np.abs(self.inboard_node.position):
            return (self.inboard_node.position + self.outboard_node.position) / 2
        else:
            return (self.outboard_node.position)
    
    @property
    def radius(self) -> float:
        return 0.625 / 2

    @property
    def height(self) -> float:
        return float(np.linalg.norm(self.inboard_node.position - self.outboard_node.position))