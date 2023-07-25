from sim.system_models.vectors.vector import Vector


class ObservablesVector(Vector):
    def __init__(self):
        super().__init__()

        self.hv_battery_open_circuit_voltage = 0
        self.hv_battery_terminal_voltage = 0
        self.lv_battery_open_circuit_voltage = 0
        self.lv_battery_terminal_voltage = 0
