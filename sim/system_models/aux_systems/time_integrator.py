from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


class TimeIntegrator:
    def __init__(self, time_step: float):
        self.time_step = time_step

    def eval(self, state: StateVector, state_dot: StateDotVector) -> StateVector:
        next_state = StateVector()

        # simple euler's method
        next_state.velocity = state.velocity + (state_dot.acceleration * self.time_step)

        return next_state
