from suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.secondary_elements.kin_rc import KinRC
import numpy as np


class Axle:
    def __init__(self, left_assy: DoubleWishbone, right_assy: DoubleWishbone) -> None:
        self.left = left_assy
        self.right = right_assy

        self.kin_RC: KinRC = KinRC(left_swing_arm=self.left.FVIC_link, right_swing_arm=self.right.FVIC_link)
        self.roll_angle = 0

        self.elements = [self.left, self.right]
    
    def roll(self, angle: float):
        roll_steps = np.arange(self.roll_angle, angle, 0.01)
        for roll_step in roll_steps:
            # Rotate relative contact patch position vector the theoretical angle
            # Pull the vertical travel from that rotation
            # Use that vertical travel as an initial guess
            # Use a binary approach (divide and multiply by 2) until the correct angle is met
            pass

    def steer(self, rack_displacement: float):
        self.left.steer(steer=rack_displacement)
        self.right.steer(steer=rack_displacement)
    
    def jounce(self, jounce: float):
        self.left.jounce(jounce=jounce)
        self.right.jounce(jounce=jounce)

    def plot_elements(self, plotter, verbose):
        if verbose:
            self.kin_RC.plot_elements
            pass
        for corner in self.elements:
            corner.plot_elements(plotter=plotter)