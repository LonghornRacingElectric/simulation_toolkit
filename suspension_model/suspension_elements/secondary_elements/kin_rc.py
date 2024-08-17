from suspension_model.suspension_elements.primary_elements.link import Link
from suspension_model.suspension_elements.primary_elements.node import Node
from suspension_model.suspension_elements.secondary_elements.cg import CG
import numpy as np


class KinRC:
    def __init__(self, left_swing_arm: Link, right_swing_arm: Link, cg: CG) -> None:
        self.left_swing_arm = left_swing_arm
        self.right_swing_arm = right_swing_arm
        self.cg = cg

        # Actual kinematic roll center
        self.true_KinRC = Node(position=self.true_KinRC_pos)
        self.lateral_position, self.vertical_position = self.true_KinRC.position[1:]

        self.cg_axis_KinRC = Node(position=self.cg_axis_KinRC_pos)
        
        self.elements = [self.true_KinRC, self.cg_axis_KinRC]

    def update(self):
        self.true_KinRC.position = self.true_KinRC_pos
        self.lateral_position, self.vertical_position = self.true_KinRC.position[1:]
        self.cg_axis_KinRC.position = self.cg_axis_KinRC_pos

    @property
    def cg_axis_KinRC_pos(self):
        y = (self.true_KinRC.position[2] - self.cg.position[2]) * self.cg.direction[1] / self.cg.direction[2] + self.cg.position[1]
        return [self.true_KinRC.position[0], y, self.true_KinRC.position[2]]

    @property
    def true_KinRC_pos(self):
        return self.left_swing_arm.yz_intersection(link=self.right_swing_arm)

    def plot_elements(self, plotter):
        for element in self.elements:
            element.plot_elements(plotter=plotter)