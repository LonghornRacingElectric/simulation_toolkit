from suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from suspension_model.suspension_elements.secondary_elements.kin_rc import KinRC
from suspension_model.suspension_elements.secondary_elements.cg import CG
from suspension_model.assets.misc_linalg import rotation_matrix
import numpy as np


class Axle:
    def __init__(self, left_assy: DoubleWishbone, right_assy: DoubleWishbone, cg: CG) -> None:
        self.left = left_assy
        self.right = right_assy
        self.cg = cg

        self.kin_RC: KinRC = KinRC(left_swing_arm=self.left.FVIC_link, right_swing_arm=self.right.FVIC_link, cg=self.cg)

        self.elements = [self.left, self.right]
    
    def roll(self, angle: float):
        self.reset_roll()
        # Rotate relative contact patch position vector the theoretical angle
        # Pull the vertical travel from that rotation
        # Use that vertical travel as an initial guess
        # Use a binary approach (divide and multiply by 2) until the correct angle is met

        roll_steps = np.linspace(0, angle, 100)

        for i, cum_roll in enumerate(roll_steps):
            if cum_roll == 0:
                continue
            
            # Step size
            roll_step = cum_roll / (i + 1)

            # Calculate relative contact patch position vectors
            left_pos = self.left.contact_patch.position - self.kin_RC.cg_axis_KinRC.position
            right_pos = self.right.contact_patch.position - self.kin_RC.cg_axis_KinRC.position

            # Calculate rotation matrices to rotate contact patches the current step size
            left_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * roll_step)
            right_rot = rotation_matrix(unit_vec=[1, 0, 0], theta=-1 * roll_step)

            # Extract jounce travel from rotation calculation as an initial guess
            # Note that track width changes, so this jounce value won't be correct
            left_guess = (np.matmul(left_rot, left_pos) + self.kin_RC.cg_axis_KinRC.position)[2]
            right_guess = (np.matmul(right_rot, right_pos) + self.kin_RC.cg_axis_KinRC.position)[2]

            # Apply jounce
            self.left.jounce(left_guess)
            self.right.jounce(right_guess)
            self.kin_RC.update()
            
            # Calculate actual roll
            roll_vec = self.left.contact_patch.position - self.right.contact_patch.position
            calc_roll = np.arctan(roll_vec[2] / roll_vec[1])

            # Adjust jounce until roll is correct within 0.1%
            while (calc_roll - cum_roll) / cum_roll > 0.001:
                # New jounce condition
                self.left.jounce_step(left_guess)
                self.right.jounce_step(right_guess)
                self.kin_RC.update()

                # Calculate roll
                roll_vec = self.left.contact_patch.position - self.right.contact_patch.position
                calc_roll = np.arctan(roll_vec[2] / roll_vec[1])
                
                # Use percent error to dyanmically size iterations on jounce
                error_metric = (calc_roll - cum_roll) / cum_roll

                # Adjust guesses
                left_guess *= error_metric
                right_guess *= error_metric
        
        print(self.left.current_jounce)
        print(self.right.current_jounce)
                
    def reset_roll(self):
        self.axle_jounce(jounce=0)
        self.kin_RC.update()

    def steer(self, rack_displacement: float):
        self.left.steer(steer=rack_displacement)
        self.right.steer(steer=rack_displacement)

        self.kin_RC.update()
    
    def axle_jounce(self, jounce: float):
        self.left.jounce(jounce=jounce)
        self.right.jounce(jounce=jounce)
        
        self.kin_RC.update()

    def plot_elements(self, plotter, verbose):
        if verbose:
            self.kin_RC.plot_elements(plotter=plotter)
        for corner in self.elements:
            corner.plot_elements(plotter=plotter)