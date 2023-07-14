"""

Some vehicle parameters are too complex to just be defined by constant values. This class allows you to use a
function as a parameter instead.

For an example, think about how a nonlinear spring's stiffness may be a function of its displacement,
or how a battery voltage may be a function of its state of charge. These relationships can be imported as data from a
CSV file, from a list of points, or from a Python function.

"""

from typing import Callable


# TODO implement

class CurveParameter:
    def __init__(self, from_csv: str = None, from_function: Callable[[float], float] = None,
                 from_points: list[tuple[float, float]] = None):
        if from_csv:
            pass
        elif from_function:
            pass
        elif from_points:
            pass
        else:
            pass

    def sample(self, x: float):
        y = x
        return y
