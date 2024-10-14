import numpy as np
from scipy import optimize
# from simulation_toolkit.vehicle_model.vehicle_model import VehicleModel
import os
import sys

# make everything in cd accessible
for dirpath, dirnames, filenames in os.walk(os.curdir):
    sys.path.append(dirpath)

"""

for now i guess i'll assume a vehicle ("car") class object that has parameter, previous state ("in"),
current state ("out"), and current control signal ("control") attributes

e.g.
ptn_parameters = car.parameters.powertrain
state_in = car.state_in
controls = car.controls
car.state_out = state_out
    sate out is just an updated copy of state in
    meaning state_out = state_in after state in has been modified by functions that take in
    state_in and controls as arguments (current control signal will result in some change in the vehicle
    state over a certain timestep for transient, or will result in a different quasi-static state (because certain
    powertrain parameters depend on the state of other parts of the car, e.g. current tire grip)

"""

class PowertrainModel:

    """
    ## Powertrain Model

    Parameters
    ----------
    # type shit

    """

    # def __init__(self, car: VehicleModel) -> None:
    def __init__(self, car) -> None:
        """
        ~ Contains all the static powertrain parameters (coefficients, constants, lookup tables, etc.) ~

        """

        self.cell_dcir = None
        self.cell_ocv = None
        self.soc = None
        self.motor_rpm = None
        self.back_emf = None

        params = car.parameters['powertrain']  # dictionary with various ptn parameters / LUTs...

        self.motor_max_rpm = params['motor_max_rpm']['value']
        self.motor_efficiency = params['motor_efficiency']['table']
        self.inverter_efficiency = params['inverter_efficiency']['table']
        self.cell_ocv_lu = params['cell_ocv_lu']['table']
        self.cell_dcir_lu = params['cell_dcir_lu']['table']
        self.s_count = params['s_count']['value']
        self.p_count = params['p_count']['value']
        self.motor_kv = params['motor_kv']['value']
        self.motor_kt = params['motor_kt']['value']
        self.motor_max_torque = params['motor_max_torque']['value']
        self.max_cell_voltage = params['max_cell_voltage']['value']
        self.max_cell_charge_current = params['max_cell_charge_current']['value']
        self.regulated_power_limit_emeter = params['regulated_power_limit_emeter']['value']
        self.mech_efficiencies = params['mech_efficiencies']['value']
        self.diff_preload = params['diff_preload']['value']
        self.diffBiasPerc = params['diffBiasPerc']['value']
        self.diffMuRatio = params['diffMuRatio']['value']
        self.gear_ratio = params['gear_ratio']['value']

        # for par in list(params.keys()):
        #     try:
        #         self.__setattr__(par, params[par]['value'])
        #     except KeyError:
        #         try:
        #             self.__setattr__(par, params[par]['table'])
        #         except KeyError:
        #             print('parameter initializing error:', par)

    def eval(self, state_in, controls):
        """
        ~ Evaluate a powertrain state given vehicle state and control input ~


        ~ Arguments ~
        -------------
        state_in: car.state_in (type = dict)
            left_wheel_speed: float [rad/s]
            right_wheel_speed: float [rad/s]
            soc: float [0->1]
            avg_cell_temp: float [C]
            tire_torques: [FL, FR, RL, RR]: list[float] [Nm]

        controls: car.controls (type = dict)
            torque_request: float [Nm]


        ~ Return ~
        ----------
        powertrain_torques: [FL, FR, RL, RR]: list[float] [Nm]

        """

        torque_request = controls['torque_request']
        left_wheel_speed = state_in['wheel_angular_velocities'][2]
        right_wheel_speed = state_in['wheel_angular_velocities'][3]
        motor_ang_vel = np.average([left_wheel_speed, right_wheel_speed])*self.gear_ratio  # differential imposed constraint
        self.motor_rpm = self.omega_to_rpm(omega=motor_ang_vel)
        self.back_emf = self.motor_rpm / self.motor_kv
        self.soc = state_in['soc']
        # cell_temp = state_in.avg_cell_temp
        self.cell_ocv = self.cell_ocv_lu([self.soc])  # add f(cell temp)
        self.cell_dcir = self.cell_dcir_lu([self.soc])  # add f(cell temp, frequency)... 2 RC model later

        def max_motor_battery_pwr_calc(test_torque: float) -> float:
            # assumption is that torque-speed curve, efficiency map are symetric about x (speed or RPM) axis
            # positive (+) power is defined in the direction from battery to motor
            # this calc uses some assumed torque and rpm to evaluate the maximum power the motor can output and is
            # later compared to the requested power to determine actual output

            if self.motor_rpm > self.motor_max_rpm:
                power = 0
                return power

            motor_ang_vel = self.rpm_to_omega(self.motor_rpm)
            motor_pwr_mech = test_torque * motor_ang_vel
            phase_current = test_torque / self.motor_kt
            dc_pwr_inverter = motor_pwr_mech / (self.motor_efficiency([self.motor_rpm, test_torque]) *
                                                self.inverter_efficiency([phase_current, self.back_emf]))
            battery_terminal_voltage = self.terminal_voltage_calc(battery_power=dc_pwr_inverter)
            # print('voltage:', battery_terminal_voltage)
            max_power_rpm = battery_terminal_voltage * self.motor_kv

            if test_torque >= 0:  # positive torque request
                power = self.motor_max_torque * self.rpm_to_omega(max_power_rpm)
            else:  # negative torque request; regen
                motor_regen_power_limit = (-self.motor_max_torque * motor_ang_vel)
                battery_charge_power_limit_voltage = (battery_terminal_voltage *
                                                      (battery_terminal_voltage-(self.cell_ocv*self.s_count))
                                                      / (self.cell_dcir/1000))
                battery_charge_power_limit_current = battery_terminal_voltage*self.max_cell_charge_current*self.p_count
                power = max(motor_regen_power_limit, battery_charge_power_limit_voltage,
                            battery_charge_power_limit_current)
                # these should all be negative quantities so take the one closest to zero. i.e. max.
            return power

        def total_power_limit_calc(torque_req: float) -> float:
            phase_current = torque_req / self.motor_kt
            regulated_power_limit_motor = (self.regulated_power_limit_emeter *
                                           self.motor_efficiency(np.array([self.motor_rpm, torque_req])) *
                                           self.inverter_efficiency([phase_current, self.back_emf]))
            if torque_req >= 0:
                total_power_limit = min(regulated_power_limit_motor, max_motor_battery_pwr_calc(torque_req))
            elif torque_req < 0:
                total_power_limit = max(-regulated_power_limit_motor, max_motor_battery_pwr_calc(torque_req))
            return total_power_limit

        power_request = torque_request * motor_ang_vel

        if abs(power_request) <= abs(total_power_limit_calc(torque_req=torque_request)):
            motor_torque = torque_request

        else:   # field weakening, emeter power regulated, or battery power regulated. need to solve for torque
            # iteratively by trying lower torques (thus lower power; rpm is constant)

            def power_residual(torque):
                # print(torque)
                pwr_req = torque * motor_ang_vel
                pwr_limit = total_power_limit_calc(float(torque))
                return np.abs(pwr_req - pwr_limit)
                # print('power request:', pwr_req)
                # print('power limit:', pwr_limit)

            motor_torque_result = optimize.minimize(power_residual, x0=torque_request, tol=0.0001)
            motor_torque = float(motor_torque_result.x)

        # evaluate final state
        state_out = state_in
        motor_power = motor_torque * motor_ang_vel
        battery_terminal_power = motor_power/(self.motor_efficiency([self.motor_rpm, motor_torque]) *
                                              self.inverter_efficiency([motor_torque/self.motor_kt, self.back_emf]))
        battery_terminal_voltage = self.terminal_voltage_calc(battery_power=battery_terminal_power)
        battery_current = battery_terminal_power / battery_terminal_voltage
        battery_power = self.cell_ocv*self.s_count * battery_current

        # assign state out
        state_out['motor_rpm'] = self.motor_rpm
        state_out['motor_torque'] = motor_torque
        state_out['motor_power'] = motor_power
        state_out['battery_terminal_power'] = battery_terminal_power
        state_out['battery_terminal_voltage'] = battery_terminal_voltage
        state_out['battery_current'] = battery_current
        state_out['battery_power'] = battery_power

        # Solve for wheel torques / differential split
        # ~ diff model ~
        def trans_eff():
            # mos_bearing_eff = 0.995
            # chain_eff = 0.98
            # diff_bearing_eff = 0.995**2  # 2 diff bearings
            mos_bearing_eff = self.mech_efficiencies['mos_bearing']
            chain_eff = self.mech_efficiencies['chain']
            diff_bearing_eff = self.mech_efficiencies['diff_bearing']
            return mos_bearing_eff*chain_eff*diff_bearing_eff  # this is bullshit, will implement later fr

        def diff_to_wheel_eff():
            # cv_joint_eff = 0.995
            # wheel_bearing_eff = 0.995
            cv_joint_eff = self.mech_efficiencies['cv_joint']
            wheel_bearing_eff = self.mech_efficiencies['wheel_bearing']
            return cv_joint_eff*wheel_bearing_eff  # this is also bullshit, will implement later fr

        def tor_max_delta(diff_torque, diffing=False):
            delta = self.diff_preload +\
                    diff_torque*self.diffBiasPerc  # make sure this definition is consistent between diffs
            if diffing:
                delta *= self.diffMuRatio  # if diffing, need to use dynamic friction coefficient (not currently used)
            return delta

        diff_in_torque = motor_torque*self.gear_ratio*trans_eff()
        left_grip = state_in['tire_torques'][2]
        right_grip = state_in['tire_torques'][3]
        grip_delta = (left_grip - right_grip)/diff_to_wheel_eff()     # tire torque delta at diff, not tire
        if np.abs(grip_delta) != 0:
            bias_dir = grip_delta/np.abs(grip_delta)  # left is pos (+), right is neg (-)
            torque_delta = min(tor_max_delta(diff_in_torque), np.abs(grip_delta))*bias_dir  # delta = left minus right
        else:
            torque_delta = 0

        left_torque = ((diff_in_torque + torque_delta)/2)*diff_to_wheel_eff()
        right_torque = ((diff_in_torque - torque_delta)/2)*diff_to_wheel_eff()

        powertrain_torques = [0, 0, left_torque, right_torque]
        state_out['torques'] = powertrain_torques  # need to implement mechanical braking!!
        # print('motor torque=', motor_torque)

        return state_out

    def omega_to_rpm(self, omega):
        return omega * 60 / (2 * np.pi)

    def rpm_to_omega(self, rpm):
        return rpm * (2 * np.pi) / 60

    def terminal_voltage_calc(self, battery_power):
        # reminder: all these power figures are in terms of the load the battery is supplying.
        # the actual power from the battery needs to account for the cell IR.
        cell_power = battery_power / (self.s_count * self.p_count)

        def terminal_voltage_func(terminal_voltage, power):
            return terminal_voltage ** 2 - terminal_voltage * self.cell_ocv + power * self.cell_dcir/1000

        max_power_transfer = self.cell_ocv ** 2 / (4 * self.cell_dcir/1000)
        if cell_power >= max_power_transfer:
            cell_terminal_voltage = (max_power_transfer * self.cell_dcir/1000) ** 0.5
        else:
            def term_volt_reformatted(v):
                return terminal_voltage_func(v, power=cell_power)

            cell_terminal_voltage_res = optimize.root(fun=term_volt_reformatted, x0=np.array(self.cell_ocv))
            cell_terminal_voltage = cell_terminal_voltage_res.x[-1]

        if cell_terminal_voltage > self.max_cell_voltage:
            cell_terminal_voltage = self.max_cell_voltage

        pack_terminal_voltage = cell_terminal_voltage * self.s_count
        return pack_terminal_voltage

    # brakes bullshit
    def _mech_brake_calcs(self, DF, PR, BB, MC_SA, C_SA, mu, RR, TR, brake_pct):
        pedal_force = DF * brake_pct

        front_MC_SA = MC_SA[0]
        rear_MC_SA = MC_SA[1]
        front_C_SA = C_SA[0]
        rear_C_SA = C_SA[1]
        front_mu = mu[0]
        rear_mu = mu[1]
        front_RR = RR[0]
        rear_RR = RR[1]
        front_TR = TR[0]
        rear_TR = TR[2]

        front_pedal_force = pedal_force * PR * BB
        rear_pedal_force = pedal_force * PR * (1 - BB)
        front_line_pressure = front_pedal_force / front_MC_SA
        rear_line_pressure = rear_pedal_force / rear_MC_SA

        front_braking_force = front_line_pressure * front_C_SA * front_mu
        rear_braking_force = rear_line_pressure * rear_C_SA * rear_mu

        front_braking_torque = -1 * front_braking_force * front_RR
        rear_braking_torque = -1 * rear_braking_force * rear_RR

        line_pressures = [front_line_pressure, rear_line_pressure]
        braking_torques = [front_braking_torque, front_braking_torque, rear_braking_torque, rear_braking_torque]

        return [line_pressures, braking_torques]


