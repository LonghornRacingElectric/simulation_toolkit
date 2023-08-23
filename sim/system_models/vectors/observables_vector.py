from sim.system_models.vectors.vector import Vector


class ObservablesVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_open_circuit_voltage = 0
        self.hv_battery_terminal_voltage = 0
        self.lv_battery_open_circuit_voltage = 0
        self.lv_battery_terminal_voltage = 0

        self.aero_forces = [0, 0, 0]
        self.aero_moments = [0, 0, 0]

        self.tire_forces_IMF = [0, 0, 0, 0]
        self.tire_moments_IMF = [0, 0, 0, 0]

        self.forces_NTB = [0, 0, 0]
        self.moments_NTB = [0, 0, 0]

        self.summation_forces = [0, 0, 0]
        self.summation_moments = [0, 0, 0]

        # Tires [FL, FR, RL, RR]
        self.slip_angles = [0, 0, 0, 0]
        self.inclination_angles = [0, 0, 0, 0]
        self.normal_loads = [0, 0, 0, 0]
        self.raw_tire_forces = [0, 0, 0, 0]
