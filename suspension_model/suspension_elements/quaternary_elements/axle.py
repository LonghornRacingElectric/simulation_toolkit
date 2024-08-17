from suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.secondary_elements.kin_rc import KinRC
from suspension_model.suspension_elements.secondary_elements.cg import CG
from suspension_model.assets.misc_linalg import rotation_matrix
from scipy.optimize import fsolve
import numpy as np


class Axle:
    def __init__(self, left_assy: DoubleWishbone, right_assy: DoubleWishbone, cg: CG) -> None:
        self.left = left_assy
        self.right = right_assy
        self.cg = cg

        self.kin_RC: KinRC = KinRC(left_swing_arm=self.left.FVIC_link, right_swing_arm=self.right.FVIC_link, cg=self.cg)

        self.elements = [self.left, self.right]
    
    def roll(self, angle: float):
        # self.reset_roll()

        if angle == 0:
            return
        
        left_cp = self.left.contact_patch
        right_cp = self.right.contact_patch

        cg_lateral_pos = self.cg.position[1] - right_cp.position[1]
        left_cp_pos = left_cp.position[1] - right_cp.position[1]

        left_arm = abs(left_cp_pos - cg_lateral_pos)
        right_arm = abs(left_cp_pos - left_arm)

        LR_ratio = left_arm / right_arm

        left_jounce_guess = left_arm * np.tan(angle)
        jounce_soln = fsolve(self._roll_resid_func, [left_jounce_guess], args=[angle, LR_ratio])

        self.left.jounce(roll_jounce=-1 * jounce_soln[0])
        self.right.jounce(roll_jounce=jounce_soln[0] / LR_ratio)

    def _roll_resid_func(self, x, args):
        angle: float = args[0]
        LR_ratio: float = args[1]
        
        left_jounce_guess = x[0]
        right_jounce_guess = x[0] / LR_ratio

        left_cp = self.left.contact_patch
        right_cp = self.right.contact_patch

        self.left.jounce(roll_jounce=-1 * left_jounce_guess)
        self.right.jounce(roll_jounce=right_jounce_guess)

        calculated_track = abs(left_cp.position[1] - right_cp.position[1])
        calculated_roll = np.arctan((left_jounce_guess + right_jounce_guess) / calculated_track)

        return [calculated_roll - angle]
                
    def reset_roll(self):
        self.axle_jounce(jounce=0)
        # self.kin_RC.update()

    def steer(self, rack_displacement: float):
        self.left.steer(steer=rack_displacement)
        self.right.steer(steer=rack_displacement)

        self.kin_RC.update()
    
    def axle_heave(self, heave: float):
        self.left.jounce(heave_jounce=heave)
        self.right.jounce(heave_jounce=heave)
    
    def axle_jounce(self, jounce: float):
        self.left.jounce(jounce=jounce)
        self.right.jounce(jounce=jounce)
        
        self.kin_RC.update()

    def plot_elements(self, plotter, verbose):
        if verbose:
            self.kin_RC.plot_elements(plotter=plotter)
        for corner in self.elements:
            corner.plot_elements(plotter=plotter)