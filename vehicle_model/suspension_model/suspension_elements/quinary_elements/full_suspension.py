from vehicle_model.suspension_model.suspension_elements.secondary_elements.kin_pc import KinPC
from vehicle_model.suspension_model.suspension_elements.quaternary_elements.axle import Axle
from vehicle_model.suspension_model.suspension_elements.secondary_elements.cg import CG
from scipy.optimize import fsolve
from typing import Sequence
import numpy as np


class FullSuspension:
    """
    ## Full Suspension

    Full suspension object

    Parameters
    ----------
    Fr_axle : Axle
        Front axle object
    Rr_axle : Axle
        Rear axle object
    cg : CG
        Center of gravity object
    """
    def __init__(self, Fr_axle: Axle, Rr_axle: Axle, cg: CG) -> None:
        self.parameters = locals().copy()
        del self.parameters['self']
        
        self.Fr_axle = Fr_axle
        self.Rr_axle = Rr_axle
        self.cg = cg
        self.transform_origin = True

        self.left_kin_PC: KinPC = KinPC(front_swing_arm=self.Fr_axle.left.SVIC_link, rear_swing_arm=self.Rr_axle.left.SVIC_link, cg=self.cg)
        self.right_kin_PC: KinPC = KinPC(front_swing_arm=self.Fr_axle.right.SVIC_link, rear_swing_arm=self.Rr_axle.right.SVIC_link, cg=self.cg)

        self.current_average_cp = (np.array(self.Fr_axle.left.contact_patch.position) \
                                   + np.array(self.Fr_axle.right.contact_patch.position) \
                                    + np.array(self.Rr_axle.left.contact_patch.position) \
                                        + np.array(self.Rr_axle.right.contact_patch.position)) / 4
        
        self.prac_average_cp = self.current_average_cp
        self.ang_x = 0
        self.ang_y = 0

        self.elements = [self.Fr_axle, self.cg]
        self.all_elements = [self.Fr_axle, self.Rr_axle]

    def steer(self, rack_displacement: float) -> None:
        """
        ## Steer

        Steers full suspension (only front axle here)

        Parameters
        ----------
        rack_displacement : float
            Lateral translation of rack

        Returns
        -------
        None
        """
        self.reset_position()
        self.Fr_axle.steer(rack_displacement=rack_displacement)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        if self.transform_origin:
            self.flatten()

    def heave(self, heave: float) -> None:
        """
        ## Heave

        Heaves full suspension

        Parameters
        ----------
        heave : float
            Vertical translation of full suspension

        Returns
        -------
        None
        """
        self.reset_position()
        self.Fr_axle.axle_heave(heave=heave)
        self.Rr_axle.axle_heave(heave=heave)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        if self.transform_origin:
            self.flatten()

    def pitch(self, angle: float) -> None:
        """
        ## Pitch

        Pitches full suspension

        Parameters
        ----------
        angle : float
            Pitch angle of full suspension in radians

        Returns
        -------
        None
        """
        self.reset_position()

        # if angle == 0:
        #     return
        
        FL_cp = self.Fr_axle.left.contact_patch
        RL_cp = self.Rr_axle.left.contact_patch

        cg_long_pos = self.cg.position[0] - RL_cp.position[0]
        front_cp_pos = FL_cp.position[0] - RL_cp.position[0]

        front_arm = abs(front_cp_pos - cg_long_pos)
        rear_arm = abs(front_cp_pos - front_arm)

        FR_ratio = front_arm / rear_arm

        front_heave_guess = front_arm * np.sin(angle)
        heave_soln = fsolve(self._pitch_resid_func, [front_heave_guess], args=[angle, FR_ratio])

        self.Fr_axle.axle_pitch(heave=heave_soln[0])
        self.Rr_axle.axle_pitch(heave=-1 * heave_soln[0] / FR_ratio)

        self.left_kin_PC.update()
        self.right_kin_PC.update()

        if self.transform_origin:
            self.flatten()
    
    def roll(self, angle: float) -> None:
        """
        ## Roll

        Rolls full suspension

        Parameters
        ----------
        angle : float
            Roll angle in radians

        Returns
        ----------
        None
        """
        self.reset_position()
        self.Fr_axle.roll(angle=angle)
        self.Rr_axle.roll(angle=angle)
        self.left_kin_PC.update()
        self.right_kin_PC.update()
        if self.transform_origin:
            self.flatten()

    @property
    def left_wheelbase(self) -> float:
        """
        ## Left Wheelbase

        Calculates and returns left wheelbase attribute

        Returns
        -------
        float
            Left wheelbase
        """
        left_wheelbase = abs(self.Fr_axle.left.contact_patch.position[0] - self.Rr_axle.left.contact_patch.position[0])

        return left_wheelbase
    
    @property
    def right_wheelbase(self) -> float:
        """
        ## Right Wheelbase

        Calculates and returns right wheelbase attribute

        Returns
        -------
        float
            Right wheelbase
        """
        right_wheelbase = abs(self.Fr_axle.right.contact_patch.position[0] - self.Rr_axle.right.contact_patch.position[0])

        return right_wheelbase

    def _pitch_resid_func(self, x, args) -> Sequence[float]:
        """
        ## Pitch Residual Function

        Calculates residual for pitch convergence

        Parameters
        ----------
        x : Sequence[float]
            Solution guess
        args : Sequence[float]
            Args for residual calculation: [angle, FR_ratio]

        Returns
        -------
        Sequence[float]
            Residuals
        """
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
    
    def _update_FAP(self) -> None:
        """
        ## Update force application points

        Updates force application points based on CG transformation

        Returns
        -------
        None
        """
        self.Fr_axle.left.FV_FAP.position = self.Fr_axle.left.FV_FAP_position
        self.Fr_axle.left.SV_FAP.position = self.Fr_axle.left.SV_FAP_position

        self.Fr_axle.right.FV_FAP.position = self.Fr_axle.right.FV_FAP_position
        self.Fr_axle.right.SV_FAP.position = self.Fr_axle.right.SV_FAP_position

        self.Rr_axle.left.FV_FAP.position = self.Rr_axle.left.FV_FAP_position
        self.Rr_axle.left.SV_FAP.position = self.Rr_axle.left.SV_FAP_position

        self.Rr_axle.right.FV_FAP.position = self.Rr_axle.right.FV_FAP_position
        self.Rr_axle.right.SV_FAP.position = self.Rr_axle.right.SV_FAP_position
    
    @property
    def heave_stiffness(self) -> float:
        """
        ## Heave Stiffness

        Calculates heave stiffness under current conditions

        Returns
        -------
        float
            Heave stiffness under current conditions
        """
        FL_wheelrate = self.Fr_axle.left.wheelrate
        FR_wheelrate = self.Fr_axle.right.wheelrate
        RL_wheelrate = self.Rr_axle.left.wheelrate
        RR_wheelrate = self.Rr_axle.right.wheelrate

        return FL_wheelrate + FR_wheelrate + RL_wheelrate + RR_wheelrate

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
        Fr_axle_rate = self.Fr_axle.roll_stiffness
        Rr_axle_rate = self.Rr_axle.roll_stiffness

        return Fr_axle_rate + Rr_axle_rate
    
    @property
    def pitch_stiffness(self) -> float:
        """
        ## Pitch Stiffness

        Calculates pitch stiffness under current conditions

        Returns
        -------
        float
            Pitch stiffness under current conditions
        """
        Fr_axle_rate = self.Fr_axle.left.wheelrate + self.Fr_axle.right.wheelrate
        Rr_axle_rate = self.Rr_axle.left.wheelrate + self.Rr_axle.right.wheelrate
        Fr_position = self.Fr_axle.left.contact_patch.position[0]
        Rr_position = self.Rr_axle.left.contact_patch.position[0]
        cg_position = self.cg.position[0]

        pitch_stiffness = 1/4 * ((Fr_position - cg_position)**2 * Fr_axle_rate + (Rr_position - cg_position)**2 * Rr_axle_rate)

        return pitch_stiffness

    def reset_position(self) -> None:
        """
        ## Reset Position

        Resets all vehicle transformations

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        # Translate all points about the origin
        self.translate(translation=-1 * self.prac_average_cp)
        
        # Undo all rotations
        self.flatten_rotate(angle=[self.ang_x, -1 * self.ang_y, 0])
        
        # Revert all points to their initial position
        self.translate(translation=self.current_average_cp)
    
    def hard_reset(self) -> None:
        """
        ## Hard Reset

        Re-initializes the suspension, wiping the current state and starting clean

        *Only use this when absolutely necessary*

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
        self.steer(rack_displacement=0.000001)
        self.heave(heave=0.000001)
        self.pitch(angle=0.000001)
        self.roll(angle=0.000001)

        self.self = FullSuspension(**self.parameters)

    def flatten(self) -> None:
        """
        ## Flatten

        Translate and rotates vehicle such that all contact patches coincide with the ground

        Parameters
        ----------
        None

        Returns
        -------
        None
        """
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
        if self.transform_origin:
            self.ang_x = ang_x
            self.ang_y = ang_y
            self.flatten_rotate(angle=[-1 * ang_x, ang_y, 0])

        # Translate all points back to their initial origin
        updated_average_cp = np.array(list(average_cp)[:2] + [0])
        self.prac_average_cp = updated_average_cp
        self.translate(translation=updated_average_cp)
    
    def plane(self, points: Sequence[Sequence[float]]) -> Sequence[float]:
        """
        ## Plane

        Calculates plane from a collection of three points
        - General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0

        Parameters
        ----------
        points : Sequence[Sequence[float]]
            Three points defining plane

        Returns
        -------
        Sequence[float]
            Parameters defining plane: [a, b, c, x_0, y_0, z_0]
        """
        assert len(points) == 3, f"Plane generator only accepts 3 points | {len(points)} points were given"
        PQ = points[1] - points[0]
        PR = points[2] - points[0]

        a, b, c = np.cross(PQ, PR)
        x_0, y_0, z_0 = points[0]

        return [a, b, c, x_0, y_0, z_0]

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
            self.left_kin_PC.plot_elements(plotter=plotter)
            self.right_kin_PC.plot_elements(plotter=plotter)
        for axle in self.elements:
            axle.plot_elements(plotter=plotter, verbose=verbose)