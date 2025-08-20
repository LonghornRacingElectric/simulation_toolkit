from src.simulations.transient._transient_helpers.states_3dof import States3DOF
from src._3_custom_libraries.simulation import Simulation
import numpy as np
from typing import Sequence

from typing import Sequence, Callable, Tuple
import numpy as np
from collections import namedtuple
import matplotlib.pyplot as plt

SimulationResult = namedtuple("SimulationResult", ["t", "X", "U", "dX", "Y"])


class Transient:
    """
    ## Transient Simulation

    Simulates vehicle dynamics over time with various model fidelities.

    Parameters
    ----------
    model_path : str
        File path to vehicle definition
    use_mode : int
        Desired use-case of:

            0) 3DOF Nonlinear
            1) 6DOF Nonlinear
            2) 10DOF Nonlinear Lateral
            3) 10DOF Nonlinear Longitudinal
            4) 14DOF Nonlinear
    """

    def __init__(self, model_path: str, use_mode: int = 1):
        if use_mode == 1:
            self.state = States3DOF(model_path=model_path)
        elif 2 <= use_mode <= 5:
            raise NotImplementedError(f"use_mode {use_mode} not implemented yet.")
        else:
            raise ValueError(f"use_mode must be in [1, 5]. Got {use_mode}.")

    def ramp_steer(
        self,
        velX: float,
        ramp_height: float,
        ramp_duration: float,
        ramp_offset: float,
        sim_duration: float,
        time_step: float,
    ) -> SimulationResult:
        """
        ## Ramp Steer

        Simulate a ramp steer input.

        Parameters
        ----------
        velX : float
            Initial longitudinal velocity (m/s)
        ramp_height : float
            Final steering input (rad)
        ramp_duration : float
            Duration of the ramp (s)
        ramp_offset : float
            Time offset before ramp begins (s)
        sim_duration : float
            Total simulation time (s)
        time_step : float
            Integration time step (s)

        Returns
        -------
        SimulationResult
            Named tuple with time history, state, control input, derivatives, and outputs
        """
        U: Callable[[float], Sequence[float]] = lambda t: [
            np.interp(t,
                      [0, ramp_offset, ramp_offset + ramp_duration, sim_duration],
                      [0, 0, ramp_height, ramp_height])
        ]
        X_0 = [0.0, 0.0, 0.0, velX, 0.0, 0.0]  # [posX, posY, psi, velX, velY, velYaw]
        return self._run_simulation(X_0, U, sim_duration, time_step)

    def _run_simulation(
        self,
        X_0: Sequence[float],
        U: Callable[[float], Sequence[float]],
        duration: float,
        time_step: float,
    ) -> SimulationResult:
        t = 0.0
        X = np.array(X_0, dtype=float)

        t_hist = [t]
        X_hist = [X.copy()]
        U_hist = [U(t)]
        dX = self.state.der_state(X, U(t))
        Y = self.state.output_state(X, dX, U(t))

        dX_hist = [dX]
        Y_hist = [Y]

        while t < duration:
            print(f"{round(t, 3)} sec" + 20 * " ", end="\r")
            dX = self.state.der_state(X, U(t))
            X += time_step * np.array(dX)
            t += time_step

            t_hist.append(t)
            X_hist.append(X.copy())
            U_hist.append(U(t))
            dX_hist.append(dX)
            Y_hist.append(self.state.output_state(X, dX, U(t)))

        return SimulationResult(
            t=np.array(t_hist),
            X=np.array(X_hist),
            U=np.array(U_hist),
            dX=np.array(dX_hist),
            Y=np.array(Y_hist),
        )


if __name__ == "__main__":
    sim = Transient(model_path="./_1_model_inputs/Nightwatch.yml", use_mode=1)

    result = sim.ramp_steer(
        velX=50 * 0.2777778, # kph to mps
        ramp_height=8.5 * np.pi / 180,
        ramp_duration=3.5,
        ramp_offset=3.5,
        sim_duration=3.5 * 3,
        time_step=0.01,
    )

    # Plot Ay
    plt.plot(result.Y[:, 7] / 9.81, result.Y[:, -1] * 180 / np.pi / 9.81)
    # plt.plot(result.t, result.Y[:, 7] / abs(sim.state.sus_data.g))
    plt.xlabel("Time (s)")
    plt.ylabel("Lateral Acceleration Ay (m/s^2)")
    plt.title("Ramp Steer Response")
    plt.grid(True)
    plt.tight_layout()
    plt.show()
