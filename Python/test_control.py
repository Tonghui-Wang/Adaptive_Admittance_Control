"""Checks for the rectified admittance controller."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import control
import robot_model


class ModelTests(unittest.TestCase):
    def test_fkine_matches_toolbox(self):
        robot = robot_model.make_robot()
        rng = np.random.default_rng(1)
        for _ in range(8):
            q = rng.normal(size=6) * 0.4
            pose, _ = robot_model.fkine(q)
            toolbox = np.asarray(robot.fkine(q).A)
            self.assertLess(np.linalg.norm(pose - toolbox), 1e-9)

    def test_jacobian_matches_toolbox(self):
        robot = robot_model.make_robot()
        rng = np.random.default_rng(2)
        for _ in range(8):
            q = rng.normal(size=6) * 0.4
            got = robot_model.jacob0(q)
            expected = robot.jacob0(q)
            self.assertLess(np.linalg.norm(got - expected), 1e-8)


class LawTests(unittest.TestCase):
    def test_contact_is_unilateral(self):
        normal = np.array([0.0, 0.0, 1.0])
        point = np.zeros(3)
        force, penetration = control.contact(np.array([0.0, 0.0, 0.01]), normal, point, 2000.0)
        self.assertEqual(force, 0.0)
        self.assertEqual(penetration, 0.0)
        force, penetration = control.contact(np.array([0.0, 0.0, -0.002]), normal, point, 2000.0)
        self.assertAlmostEqual(penetration, 0.002)
        self.assertAlmostEqual(force, 4.0)

    def test_pso_finds_quadratic_minimum(self):
        def cost(x):
            return float((x[0] - 3.0) ** 2 + (x[1] + 1.0) ** 2)

        best, value = control.pso_minimize(
            cost,
            np.array([0.0, 0.0]),
            np.array([-5.0, -5.0]),
            np.array([5.0, 5.0]),
            np.array([0.8, 0.8]),
            particles=16,
            iterations=30,
            rng=np.random.default_rng(0),
        )
        self.assertLess(np.linalg.norm(best - np.array([3.0, -1.0])), 0.15)
        self.assertLess(value, 0.05)

    def test_rate_limit(self):
        got = control.rate_limit(np.array([10.0, -10.0]), np.array([0.0, 0.0]), np.array([0.35, 4.0]))
        np.testing.assert_allclose(got, np.array([0.35, -4.0]))

    def test_planned_velocity_is_finite_at_the_endpoint(self):
        robot = robot_model.make_robot()
        reference = control.plan(robot, control.default_via_points())
        self.assertEqual(reference["qd"].shape, reference["q"].shape)
        self.assertLess(np.max(np.abs(reference["qd"][-1])), 1e-8)
        self.assertTrue(np.all(np.isfinite(reference["qdd"])))
        self.assertLess(np.max(np.abs(reference["qdd"][-1])), 1e-6)


class ClosedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_model.make_robot()
        cls.reference = control.plan(cls.robot, control.default_via_points())
        cls.normal, cls.plane = control.fit_plane(cls.reference["p"])
        cls.switch = 0.5 * float(cls.reference["t"][-1])
        cls.adaptive = control.simulate(
            cls.robot, cls.reference, cls.normal, cls.plane, mode="adaptive", seed=7
        )
        cls.heavy = control.simulate(
            cls.robot, cls.reference, cls.normal, cls.plane, mode="heavy", seed=7
        )
        cls.light = control.simulate(
            cls.robot, cls.reference, cls.normal, cls.plane, mode="light", seed=7
        )

    def test_starts_in_free_space_then_holds_force(self):
        self.assertEqual(self.adaptive["penetration"][0], 0.0)
        self.assertEqual(self.adaptive["force"][0], 0.0)
        self.assertLess(self._rmse(self.adaptive, 1.0, self.switch), 0.25)
        self.assertGreater(
            self._rmse(self.heavy, 1.0, self.switch),
            self._rmse(self.adaptive, 1.0, self.switch),
        )

    def test_stiffness_estimate_follows_the_jump(self):
        before = int(2.5 / control.DT)
        self.assertAlmostEqual(self.adaptive["k_hat"][before], control.K_SOFT, delta=40.0)
        self.assertAlmostEqual(self.adaptive["k_hat"][-1], control.K_STIFF, delta=40.0)

    def test_recovery_beats_mistuned_fixed_gains(self):
        adaptive = self._rmse(self.adaptive, self.switch, self.switch + 1.0)
        self.assertLess(adaptive, self._rmse(self.heavy, self.switch, self.switch + 1.0) - 1.0)
        self.assertLess(adaptive, self._rmse(self.light, self.switch, self.switch + 1.0))

    def test_tangent_tracking_and_gain_limits(self):
        tangent = float(np.sqrt(np.mean(self.adaptive["tangent"] ** 2)))
        self.assertLess(tangent, 0.003)
        self.assertLessEqual(np.max(np.abs(np.diff(self.adaptive["m"]))), control.RATE_LIMIT[0] + 1e-9)
        self.assertLessEqual(np.max(np.abs(np.diff(self.adaptive["b"]))), control.RATE_LIMIT[1] + 1e-9)
        # Steady contact must not retune every cycle.
        hold = slice(int(1.0 / control.DT), int(2.8 / control.DT))
        self.assertLess(np.max(self.adaptive["m"][hold]) - np.min(self.adaptive["m"][hold]), 1e-9)
        self.assertLess(np.max(self.adaptive["b"][hold]) - np.min(self.adaptive["b"][hold]), 1e-9)

    def _rmse(self, result, start, stop):
        mask = (result["t"] >= start) & (result["t"] < stop)
        error = result["force"][mask] - result["force_des"][mask]
        return float(np.sqrt(np.mean(error ** 2)))


if __name__ == "__main__":
    unittest.main()
