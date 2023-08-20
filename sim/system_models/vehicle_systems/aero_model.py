from sim.system_models.vehicle_systems.vehicle_system_model import VehicleSystemModel

from sim.model_parameters.cars.car import Car
from sim.system_models.vectors.controls_vector import ControlsVector
from sim.system_models.vectors.observables_vector import ObservablesVector
from sim.system_models.vectors.state_vector import StateVector
from sim.system_models.vectors.state_dot_vector import StateDotVector

import numpy as np


# TODO robert pls implement <3

class AeroModel(VehicleSystemModel):
    def __init__(self):
        super().__init__()

        self.controls_in = [
        ]

        self.state_in = [
            "velocity",
            "heave",
            "pitch",
            "roll",
            "body_slip"
        ]

        self.state_out = [
        ]

        self.observables_out = [
            "aero_forces",
            "aero_moments"
        ]

    def eval(self, vehicle_parameters: Car, controls_vector: ControlsVector, state_in_vector: StateVector,
             state_out_vector: StateDotVector, observables_vector: ObservablesVector):
        
        velocity = state_in_vector.velocity
        body_slip = state_in_vector.body_slip
        heave = state_in_vector.heave
        pitch = state_in_vector.pitch
        roll = state_in_vector.roll

        aero_forces, aero_moments = self._get_loads(vehicle_parameters, velocity, body_slip, heave, pitch, roll)

        observables_vector.aero_forces = aero_forces
        observables_vector.aero_moments = aero_moments

        pass

    def _get_loads(self, vehicle_parameters: Car, speed: float, body_slip: float, heave: float, pitch: float, roll: float):
        # Conversion factors
        rad_to_deg = 180 / np.pi

        # Convert angles to degrees for sensitivity calcs
        body_slip *= rad_to_deg
        pitch *= rad_to_deg
        roll *= rad_to_deg

        # Gets aero coefficients for each component
        ClA = vehicle_parameters.ClA_tot * np.array(vehicle_parameters.ClA_dist)
        CdA = vehicle_parameters.CdA_tot * np.array(vehicle_parameters.CdA_dist)
        CsA = vehicle_parameters.CsA_tot * np.array(vehicle_parameters.CsA_dist)

        # Converts from CAD origin to IMF
        CoP_IMF = np.array(vehicle_parameters.CoP)
        CoP_IMF[:,0] += vehicle_parameters.cg_bias * vehicle_parameters.wheelbase

        coeffs = np.array([ClA, CdA, CsA])

        p_dir = 1 if pitch <= 0 else 0
        psens = np.array(vehicle_parameters.p_sens)[:,:,p_dir]

        angles = np.array([abs(body_slip), abs(pitch), abs(roll)])
        angle_sens = np.array([vehicle_parameters.bs_sens, psens, vehicle_parameters.r_sens])
        angle_sens /= 100   # convert from percentages

        s_dir = -1
        if body_slip < 0:
            s_dir = 1

        # Multiply each sensitivity by the corresponding angle
        # repeat -> reshape turns angles array into 3x3x3 array with angles in
        # the right place for element-wise multiplication
        angle_sens *= np.reshape(np.repeat(angles,9),(3,3,3))

        # add 1 to turn percent increases into multiplication factor
        angle_sens += 1

        # multiply all sensitivities for given part and force together
        angle_sens = np.prod(angle_sens, axis = 0)

        # multiply lift, drag, and sideforce coefficients by sensitivities
        coeffs = angle_sens * coeffs.T

        # heave sensitivities
        heave_sens = self._get_heave_sens(vehicle_parameters, heave)
        coeffs *= heave_sens

        # calculate force arrays for each direction: F_part = [front, undertray, rear]
        Fl_part = 0.5 * vehicle_parameters.air_density * coeffs[:,0] * speed ** 2
        Fd_part = 0.5 * vehicle_parameters.air_density * coeffs[:,1] * speed ** 2
        Fs_part = 0.5 * vehicle_parameters.air_density * coeffs[:,2] * (speed * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir

        # drag is x, sideforce is y, lift is z
        part_force = np.array([-Fd_part, Fs_part, -Fl_part])

        # forces are sum of forces on each part
        forces = np.array([-np.sum(Fd_part), np.sum(Fs_part), -np.sum(Fl_part)])

        # sum moments from front, undertray, and rear
        moments = np.cross(CoP_IMF[0], part_force.T[0]) \
                + np.cross(CoP_IMF[1], part_force.T[1]) \
                + np.cross(CoP_IMF[2], part_force.T[2])


        # account for drag and sideforce from rest of car
        drag_no_aero = 0.5 * vehicle_parameters.air_density * vehicle_parameters.CdA0 * speed ** 2
        sideforce_no_aero = 0.5 * vehicle_parameters.air_density * vehicle_parameters.CsA0 * (speed * np.tan(body_slip/rad_to_deg)) ** 2 * s_dir
        forces += np.array([-drag_no_aero, sideforce_no_aero, 0])

        return forces, moments
    
    def _get_heave_sens(self, vehicle_parameters: Car, heave: float):
        cl_heave_sens = np.polyval(vehicle_parameters.h_sens_coefficients[0], heave)
        cd_heave_sens = np.polyval(vehicle_parameters.h_sens_coefficients[1], heave)
        
        # sens for undertraying, using it as an estimate for front wing
        heave_sens = np.array([[cl_heave_sens, cd_heave_sens, 1],
                              [cl_heave_sens, cd_heave_sens, 1],
                              [1, 1, 1]])
        
        return heave_sens