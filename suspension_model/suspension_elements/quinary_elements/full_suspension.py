from suspension_model.suspension_elements.secondary_elements.kin_pc import KinPC
from suspension_model.suspension_elements.quaternary_elements.axle import Axle
from suspension_model.suspension_elements.secondary_elements.cg import CG
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class FullSuspension:
    def __init__(self, Fr_axle: Axle, Rr_axle: Axle, cg: CG) -> None:
        self.Fr_axle = Fr_axle
        self.Rr_axle = Rr_axle
        self.cg = cg

        self.left_kin_PC: KinPC = KinPC(front_swing_arm=self.Fr_axle.left.SVIC_link, rear_swing_arm=self.Rr_axle.left.SVIC_link, cg=self.cg)
        self.right_kin_PC: KinPC = KinPC(front_swing_arm=self.Fr_axle.right.SVIC_link, rear_swing_arm=self.Rr_axle.right.SVIC_link, cg=self.cg)

        self.current_average_cp = (np.array(self.Fr_axle.left.contact_patch.position) \
                                   + np.array(self.Fr_axle.right.contact_patch.position) \
                                    + np.array(self.Rr_axle.left.contact_patch.position) \
                                        + np.array(self.Rr_axle.right.contact_patch.position)) / 4
        
        self.prac_average_cp = self.current_average_cp
        self.ang_x = 0
        self.ang_y = 0

        self.elements = [self.Fr_axle, self.Rr_axle, self.cg]
        self.all_elements = [self.Fr_axle, self.Rr_axle, self.cg,self.left_kin_PC, self.right_kin_PC]

    def steer(self, rack_displacement: float):
        self.reset_position()
        self.Fr_axle.steer(rack_displacement=rack_displacement)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        self.flatten()

    def heave(self, heave: float):
        self.reset_position()
        self.Fr_axle.axle_heave(heave=heave)
        self.Rr_axle.axle_heave(heave=heave)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        self.flatten()

    def pitch(self, angle: float):
        self.reset_position()
        angle *= np.pi / 180

        if angle == 0:
            return
        
        FL_cp = self.Fr_axle.left.contact_patch
        RL_cp = self.Rr_axle.left.contact_patch

        cg_long_pos = self.cg.position[0] - RL_cp.position[0]
        front_cp_pos = FL_cp.position[0] - RL_cp.position[0]

        front_arm = abs(front_cp_pos - cg_long_pos)
        rear_arm = abs(front_cp_pos - front_arm)

        FR_ratio = front_arm / rear_arm

        front_heave_guess = front_arm * np.tan(angle)
        heave_soln = fsolve(self._pitch_resid_func, [front_heave_guess], args=[angle, FR_ratio])

        self.Fr_axle.axle_pitch(heave=heave_soln[0])
        self.Rr_axle.axle_pitch(heave=-1 * heave_soln[0] / FR_ratio)

        self.left_kin_PC.update()
        self.right_kin_PC.update()

        self.flatten()
    
    def roll(self, angle: float):
        self.reset_position()
        self.Fr_axle.roll(angle=angle)
        self.Rr_axle.roll(angle=angle)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        self.flatten()

    def _pitch_resid_func(self, x, args):
        angle: float = args[0]
        FR_ratio: float = args[1]
        
        front_heave_guess = x[0]
        rear_heave_guess = x[0] / FR_ratio

        FL_cp = self.Fr_axle.left.contact_patch
        RL_cp = self.Rr_axle.left.contact_patch

        self.Fr_axle.axle_pitch(heave=front_heave_guess)
        self.Rr_axle.axle_pitch(heave=-1 * front_heave_guess / FR_ratio)

        calculated_wheelbase = abs(FL_cp.position[0] - RL_cp.position[0])
        calculated_pitch = np.arctan((front_heave_guess + rear_heave_guess) / calculated_wheelbase)

        return [calculated_pitch - angle]

    def reset_position(self):
        # Translate all points about the origin
        self.translate(translation=-1 * self.prac_average_cp)
        
        # Undo all rotations
        self.flatten_rotate(angle=[self.ang_x, -1 * self.ang_y, 0])
        
        # Revert all points to their initial position
        self.translate(translation=self.current_average_cp)

    def flatten(self):
        FL_cp = self.Fr_axle.left.contact_patch
        FR_cp = self.Fr_axle.right.contact_patch
        RL_cp = self.Rr_axle.left.contact_patch
        RR_cp = self.Rr_axle.right.contact_patch

        average_cp = (np.array(FL_cp.position) + np.array(FR_cp.position) + np.array(RL_cp.position) + np.array(RR_cp.position)) / 4

        self.current_average_cp = average_cp

        # Translate all points about the origin
        self.translate(translation=-1 * average_cp)
        
        # Rotate all points so contact patches are coincident with the ground
        a, b, c, x_0, y_0, z_0 = self.plane(points=[FL_cp.position, FR_cp.position, RL_cp.position])
        plane_eqn = lambda args: ((a * x_0 + b * y_0 + c * z_0) - a * args[0] - b * args[1]) / c
        ang_x = np.arctan(plane_eqn(args=[0, 1]))
        ang_y = np.arctan(plane_eqn(args=[1, 0]))
        self.ang_x = ang_x
        self.ang_y = ang_y
        self.flatten_rotate(angle=[-1 * ang_x, ang_y, 0])

        # Translate all points back to their initial origin
        updated_average_cp = np.array(list(average_cp)[:2] + [0])
        self.prac_average_cp = updated_average_cp
        self.translate(translation=updated_average_cp)

        # TODO:
        # 1. calculate the average point of all contact patches
        # completed
        # 2. shift all points vertically by the distance from the average contact patch to the ground
        # completed
        # 3. apply rotations about x and y of the average contact patch so all contact patches sit on the ground
        #   - This requires subtraction of the average contact patch from all points, applying rotations, then adding that position back
        #
    
    def plane(self, points: Sequence[float]):
        assert len(points) == 3, f"Plane generator only accepts 3 points | {len(points)} points were given"
        # General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
        PQ = points[1] - points[0]
        PR = points[2] - points[0]

        a, b, c = np.cross(PQ, PR)
        x_0, y_0, z_0 = points[0]

        return [a, b, c, x_0, y_0, z_0]

    def translate(self, translation: Sequence[float]):
        for element in self.all_elements:
            element.translate(translation=translation)
    
    def flatten_rotate(self, angle: Sequence[float]):
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)

    def plot_elements(self, plotter, verbose):
        if verbose:
            self.left_kin_PC.plot_elements(plotter=plotter)
            self.right_kin_PC.plot_elements(plotter=plotter)
        for axle in self.elements:
            axle.plot_elements(plotter=plotter, verbose=verbose)
