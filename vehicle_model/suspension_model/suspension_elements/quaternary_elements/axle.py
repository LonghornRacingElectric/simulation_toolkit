from vehicle_model.suspension_model.suspension_elements.tertiary_elements.double_wishbone import DoubleWishbone
from vehicle_model.suspension_model.suspension_elements.secondary_elements.kin_rc import KinRC
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class Axle:
    """
    ## Axle

    Axle object

    Parameters
    ----------
    left_assy : DoubleWishbone
        Left double wishbone object
    right_assy : DoubleWishbone
        Right double wishbone object
    cg : CG
        Center of gravity object
    """
    def __init__(self, left_assy: DoubleWishbone, right_assy: DoubleWishbone, cg: CG) -> None:
        self.left = left_assy
        self.right = right_assy
        self.cg = cg

        self.kin_RC: KinRC = KinRC(left_swing_arm=self.left.FVIC_link, right_swing_arm=self.right.FVIC_link, cg=self.cg)

        self.elements = [self.left, self.right]
        self.all_elements = [self.left, self.right, self.kin_RC]

    def roll(self, angle: float) -> None:
        """
        ## Roll

        Rolls axle

        Parameters
        ----------
        angle : float
            Roll angle in radians

        Returns
        ----------
        None
        """
        # if angle == 0:
        #     return
        
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

        self.kin_RC.update()

    def _roll_resid_func(self, x, args) -> Sequence[float]:
        """
        ## Roll Residual Function

        Calculates residual for roll convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution guess
        args : Sequence[float]
            Args for residual calculation: [angle, LR_ratio]

        Returns
        -------
        Sequence[float]
            Residuals
        """
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
                
    def reset_roll(self) -> None:
        """
        ## Reset Roll

        Resets roll angle

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.axle_jounce(jounce=0)
        # self.kin_RC.update()

    def steer(self, rack_displacement: float) -> None:
        """
        ## Steer

        Steers axle

        Parameters
        ----------
        rack_displacement : float
            Lateral translation of rack

        Returns
        -------
        None
        """
        self.left.steer(steer=rack_displacement)
        self.right.steer(steer=rack_displacement)

        self.kin_RC.update()
    
    def axle_heave(self, heave: float) -> None:
        """
        ## Axle Heave

        Heaves axle

        Parameters
        ----------
        heave : float
            Vertical translation of axle

        Returns
        -------
        None
        """
        self.left.jounce(heave_jounce=heave)
        self.right.jounce(heave_jounce=heave)

        self.kin_RC.update()
    
    def axle_pitch(self, heave: float) -> None:
        """
        ## Axle Pitch

        Pitches axle

        Parameters
        ----------
        heave : float
            Resulting heave from some pitch angle
            - Calculated in and received from FullSuspension object

        Returns
        -------
        None
        """
        self.left.jounce(pitch_jounce=heave)
        self.right.jounce(pitch_jounce=heave)

        self.kin_RC.update()
    
    @property
    def roll_stiffness(self) -> float:
        """
        ## Roll Stiffness

        Calculates roll stiffness under current conditions

        Returns
        -------
        float
            Roll stiffness under current conditions
        """
        left_wheelrate = self.left.wheelrate
        right_wheelrate = self.right.wheelrate
        left_position = self.left.contact_patch.position
        right_position = self.right.contact_patch.position
        cg_position = self.cg.position

        roll_stiffness = 1/4 * ((left_position[1] - cg_position[1])**2 * left_wheelrate + (right_position[1] - cg_position[1])**2 * right_wheelrate)

        return roll_stiffness
    
    def translate(self, translation: Sequence[float]) -> None:
        """
        ## Translate

        Translates all children

        Parameters
        ----------
        translation : Sequence[float]
            Translation to apply
            - Takes the form [x_shift, y_shift, z_shift]
        """
        for element in self.all_elements:
            element.translate(translation=translation)
    
    def flatten_rotate(self, angle: Sequence[float]) -> None:
        """
        ## Flatten Rotate

        Rotates all children
        - Used to re-orient vehicle such that contact patches intersect with x-y plane

        Parameters
        ----------
        angle : Sequence[float]
            Angles of rotation in radians [x_rot, y_rot, z_rot]
        """
        for element in self.all_elements:
            element.flatten_rotate(angle=angle)

    def plot_elements(self, plotter, verbose) -> None:
        """
        ## Plot Elements

        Plots all child elements

        Parameters
        ----------
        plotter : pv.Plotter
            Plotter object
        """
        if verbose:
            self.kin_RC.plot_elements(plotter=plotter)
        for corner in self.elements:
            corner.plot_elements(plotter=plotter)