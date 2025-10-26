from src.simulations.comp_eval.comp_eval import CompEval
from unittest import TestCase
import numpy as np


class TestGGVEnvelope(TestCase):
    def setUp(self):
        self.comp_eval = CompEval.__new__(CompEval)
        self.comp_eval.V_MAX = 31.1
        self.comp_eval.X_OFFSET = 0.5
        self.comp_eval.SCALE_FACTOR = 0.01

    def test_never_closes(self):
        """radius^2 > 0 for all v in [0, V_MAX]"""
        velocities = np.linspace(0.1, self.comp_eval.V_MAX, 50)

        for v in velocities:
            radius_squared = 9 * (self.comp_eval.SCALE_FACTOR * v**2 - v**3 / 4000.0)
            self.assertGreater(radius_squared, 0,
                               f"Envelope closed at v={v:.2f} m/s")

    def test_forms_loop(self):
        """Envelope first point ≈ last point"""
        velocities = [5, 10, 15, 20, 25, 30]

        for v in velocities:
            if v > self.comp_eval.V_MAX:
                continue

            ax_env, ay_env = self.comp_eval.ggv_envelope(v)

            self.assertAlmostEqual(ax_env[0], ax_env[-1], places=1)
            self.assertAlmostEqual(ay_env[0], ay_env[-1], places=1)


class TestGGVBounds(TestCase):
    def setUp(self):
        self.comp_eval = CompEval.__new__(CompEval)
        self.comp_eval.V_MAX = 31.1
        self.comp_eval.X_OFFSET = 0.5
        self.comp_eval.SCALE_FACTOR = 0.01

    def test_ax_positive(self):
        """ax_max > 0 for all v"""
        velocities = np.linspace(0.1, self.comp_eval.V_MAX, 50)

        for v in velocities:
            ax = self.comp_eval.ggv_max_long_accel(v)
            self.assertGreater(ax, 0, f"ax not positive at v={v:.2f} m/s")

    def test_ay_positive(self):
        """ay_max > 0 for all v"""
        velocities = np.linspace(0.1, self.comp_eval.V_MAX, 50)

        for v in velocities:
            ay = self.comp_eval.ggv_max_lat_accel(v)
            self.assertGreater(ay, 0, f"ay not positive at v={v:.2f} m/s")

    def test_within_circle(self):
        """(ax_g + X_OFFSET)^2 + ay_g^2 = radius²"""
        velocities = [5, 10, 15, 20, 25, 30]
        directions = np.linspace(0, 2*np.pi, 16)

        for v in velocities:
            if v > self.comp_eval.V_MAX:
                continue

            radius_squared = 9 * (self.comp_eval.SCALE_FACTOR * v**2 - v**3 / 4000.0)
            radius = np.sqrt(radius_squared)

            for direction in directions:
                ax, ay = self.comp_eval.ggv_max_accel(v, direction)
                ax_g, ay_g = ax / 9.81, ay / 9.81

                actual_r2 = (ax_g + self.comp_eval.X_OFFSET)**2 + ay_g**2

                self.assertAlmostEqual(actual_r2, radius**2, places=3,
                                       msg=f"Outside envelope at v={v:.2f}, θ={np.degrees(direction):.0f}°")


class TestGGVEdgeCases(TestCase):
    def setUp(self):
        self.comp_eval = CompEval.__new__(CompEval)
        self.comp_eval.V_MAX = 31.1
        self.comp_eval.X_OFFSET = 0.5
        self.comp_eval.SCALE_FACTOR = 0.01

    def test_min_velocity(self):
        """v = 0.1 m/s"""
        ax = self.comp_eval.ggv_max_long_accel(0.1)
        ay = self.comp_eval.ggv_max_lat_accel(0.1)

        self.assertGreater(ax, 0)
        self.assertGreater(ay, 0)

    def test_max_velocity(self):
        """v = V_MAX"""
        ax = self.comp_eval.ggv_max_long_accel(self.comp_eval.V_MAX)
        ay = self.comp_eval.ggv_max_lat_accel(self.comp_eval.V_MAX)

        self.assertGreater(ax, 0)
        self.assertGreater(ay, 0)

    def test_no_nan(self):
        """Never returns NaN"""
        velocities = np.linspace(0.1, self.comp_eval.V_MAX + 5, 50)

        for v in velocities:
            ax = self.comp_eval.ggv_max_long_accel(v)
            ay = self.comp_eval.ggv_max_lat_accel(v)

            self.assertFalse(np.isnan(ax))
            self.assertFalse(np.isnan(ay))