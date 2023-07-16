from sim.system_models.vectors.state_dot_vector import StateDotVector
from sim.system_models.vectors.state_vector import StateVector


def integrate(state: StateVector, state_dot: StateDotVector, dt: float) -> StateVector:
    next_state = StateVector()

    # simple euler's method
    next_state.velocity = state.velocity + (state_dot.acceleration * dt)

    return next_state
