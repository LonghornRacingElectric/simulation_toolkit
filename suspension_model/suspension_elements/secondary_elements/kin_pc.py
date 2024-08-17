from suspension_model.suspension_elements.primary_elements.link import Link
from suspension_model.suspension_elements.primary_elements.node import Node
from suspension_model.suspension_elements.secondary_elements.cg import CG
from typing import Sequence


class KinPC:
    def __init__(self, front_swing_arm: Link, rear_swing_arm: Link, cg: CG) -> None:
        self.front_swing_arm = front_swing_arm
        self.rear_swing_arm = rear_swing_arm
        self.cg = cg

        # Actual kinematic roll center
        self.true_KinPC = Node(position=self.true_KinPC_pos)
        self.long_position, self.vertical_position = self.true_KinPC.position[0], self.true_KinPC.position[2]

        self.cg_axis_KinPC = Node(position=self.cg_axis_KinPC_pos)
        
        # self.elements = [self.true_KinPC, self.cg_axis_KinPC]
        self.elements = [self.true_KinPC]
        # self.all_elements = [self.true_KinPC, self.cg_axis_KinPC]
        self.all_elements = [self.true_KinPC]

    def update(self):
        self.true_KinPC.position = self.true_KinPC_pos
        self.long_position, self.vertical_position = self.true_KinPC.position[0], self.true_KinPC.position[2]
        self.cg_axis_KinPC.position = self.cg_axis_KinPC_pos

    @property
    def cg_axis_KinPC_pos(self):
        x = (self.true_KinPC.position[2] - self.cg.position[2]) * self.cg.direction[0] / self.cg.direction[2] + self.cg.position[0]
        return [x, self.true_KinPC.position[1], self.true_KinPC.position[2]]

    @property
    def true_KinPC_pos(self):
        return self.front_swing_arm.xz_intersection(link=self.rear_swing_arm)
    
    def translate(self, translation: Sequence[float]):
        for element in self.all_elements:
            element.translate(translation=translation)
    
    def flatten_rotate(self, angle: Sequence[float]):
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)

    def plot_elements(self, plotter):
        for element in self.elements:
            element.plot_elements(plotter=plotter)