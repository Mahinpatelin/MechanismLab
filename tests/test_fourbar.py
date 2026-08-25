import math
import unittest

import numpy as np

from mechanism_lab import FourBar, PositionError


class FourBarTests(unittest.TestCase):
    def setUp(self):
        self.linkage = FourBar(100, 35, 110, 80)

    def test_mobility_and_grashof(self):
        self.assertEqual(self.linkage.mobility, 1)
        self.assertTrue(self.linkage.grashof)

    def test_position_closes_both_loops(self):
        state = self.linkage.solve(math.radians(40))
        a, b = np.array(state.point_a), np.array(state.point_b)
        self.assertAlmostEqual(np.linalg.norm(a), self.linkage.crank, places=9)
        self.assertAlmostEqual(np.linalg.norm(b - a), self.linkage.coupler, places=9)
        self.assertAlmostEqual(np.linalg.norm(b - np.array((self.linkage.ground, 0))), self.linkage.rocker, places=9)

    def test_velocity_matches_finite_difference(self):
        theta = 0.8
        omega = 1.7
        dt = 1e-6
        state = self.linkage.solve(theta, omega=omega)
        next_state = self.linkage.solve(theta + omega * dt, omega=omega)
        numerical = (np.array(next_state.point_b) - np.array(state.point_b)) / dt
        np.testing.assert_allclose(numerical, state.point_b_velocity, rtol=2e-5, atol=2e-5)

    def test_coupler_point_endpoints(self):
        state = self.linkage.solve(1.0)
        np.testing.assert_allclose(self.linkage.coupler_point(state, 0), state.point_a)
        np.testing.assert_allclose(self.linkage.coupler_point(state, 1), state.point_b)

    def test_invalid_geometry_at_angle(self):
        linkage = FourBar(100, 10, 20, 20)
        with self.assertRaises(PositionError):
            linkage.solve(0)

    def test_invalid_lengths(self):
        with self.assertRaises(ValueError):
            FourBar(100, 0, 50, 50)


if __name__ == "__main__":
    unittest.main()
