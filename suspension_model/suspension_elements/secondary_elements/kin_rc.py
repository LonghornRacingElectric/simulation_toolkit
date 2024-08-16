from suspension_model.suspension_elements.primary_elements.link import Link
from suspension_model.suspension_elements.primary_elements.node import Node
from suspension_model.suspension_elements.secondary_elements.cg import CG
import numpy as np


class KinRC(Node):
    def __init__(self, left_swing_arm: Link, right_swing_arm: Link, cg: CG) -> None:
        self.left_swing_arm = left_swing_arm
        self.right_swing_arm = right_swing_arm
        self.cg = cg

        self.position = self.true_KinRC_pos

        self.lateral_position, self.vertical_position = self.position[1:]

        self.roll_axis_KinRC = self.prac_KinRC_pos

    def update(self):
        self.position = self.true_KinRC_pos
        self.lateral_position, self.vertical_position = self.position[1:]
        self.roll_axis_KinRC = self.prac_KinRC_pos

    @property
    def prac_KinRC_pos(self):
        y = (self.position[2] - self.cg.position[2]) * self.cg.direction[1] / self.cg.direction[2] + self.cg.position[1]
        return [self.position[0], y, self.position[2]]

    @property
    def true_KinRC_pos(self):
        return self.left_swing_arm.yz_intersection(link=self.right_swing_arm)