from sim.model_parameters.parameters import CurveParameter


class TestCurveParameter:
    def test_from_function(self):
        def f(x):
            return x ** 2

        curve = CurveParameter(from_function=f, x_min=-2, x_max=2, x_samples=5)

        assert curve.is_set()

        assert curve.get()(-2) == 4
        assert curve.get()(-1.5) == 2.5
        assert curve.get()(-1) == 1
        assert curve.get()(-0.5) == 0.5
        assert curve.get()(0) == 0
        assert curve.get()(0.5) == 0.5
        assert curve.get()(1) == 1
        assert curve.get()(1.5) == 2.5
        assert curve.get()(2) == 4
