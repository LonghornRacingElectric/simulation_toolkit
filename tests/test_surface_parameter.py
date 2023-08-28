from sim.model_parameters.parameters import SurfaceParameter

import os


class TestSurfaceParameter:
    def test_from_function(self):
        def f(x, y):
            return x**2 + y**2

        surface = SurfaceParameter(from_function=f, x_min=-2, x_max=2, x_samples=5, y_min=-2, y_max=2, y_samples=5)

        assert surface.is_set()

        assert surface.get()(-2, -2) == 8
        assert surface.get()(-1, -2) == 5
        assert surface.get()(-1.5, -2) == 6.5
        assert surface.get()(-2, -1) == 5
        assert surface.get()(-2, -1.5) == 6.5
        assert surface.get()(-1, -1) == 2
        assert surface.get()(-1.5, -1.5) == 5

    def test_emrax(self):
        surface = SurfaceParameter(from_csv='car/emrax/Eff208.csv')
        assert surface.get()(0, 100) == 0.82
        assert surface.get()(2000, 100) == 0.95
        assert surface.get()(5500, 100) == 0.94
