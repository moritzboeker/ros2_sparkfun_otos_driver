# Copyright 2026 Moritz Emanuel Boeker
# SPDX-License-Identifier: MIT
"""Hardware-independent unit tests for the OTOS driver helpers."""

import math

import qwiic_otos

from sparkfun_otos_driver.otos_node import (
    diagonal_covariance,
    drifted_covariance,
    quaternion_from_yaw,
    body_velocity_from_odom,
    UNMEASURED_VARIANCE,
)


def test_quaternion_identity():
    """Zero yaw must map to the identity quaternion."""
    q = quaternion_from_yaw(0.0)
    assert (q.x, q.y, q.z, q.w) == (0.0, 0.0, 0.0, 1.0)


def test_quaternion_quarter_turn():
    """A 90 degree yaw maps to the expected z rotation."""
    q = quaternion_from_yaw(math.pi / 2.0)
    assert math.isclose(q.z, math.sin(math.pi / 4.0), rel_tol=1e-9)
    assert math.isclose(q.w, math.cos(math.pi / 4.0), rel_tol=1e-9)
    assert q.x == 0.0 and q.y == 0.0


def test_quaternion_is_normalized():
    """Quaternions must have unit norm for arbitrary yaw angles."""
    for yaw in (-math.pi, -1.0, 0.5, math.pi, 4.0):
        q = quaternion_from_yaw(yaw)
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        assert math.isclose(norm, 1.0, rel_tol=1e-9)


def test_diagonal_covariance_layout():
    """Variances land on the diagonal, unmeasured axes get a large value."""
    cov = diagonal_covariance(0.1, 0.2, 0.3)
    assert len(cov) == 36
    assert cov[0] == 0.1    # x
    assert cov[7] == 0.2    # y
    assert cov[35] == 0.3   # yaw
    for idx in (14, 21, 28):  # z, roll, pitch
        assert cov[idx] == UNMEASURED_VARIANCE
    off_diagonal = [v for i, v in enumerate(cov) if i not in (0, 7, 14, 21, 28, 35)]
    assert all(v == 0.0 for v in off_diagonal)


def test_drifted_covariance_at_origin_equals_base():
    """With zero distance traveled, only the base variance is published."""
    cov = drifted_covariance([0.1, 0.2, 0.3], [0.005, 0.005, 0.005], 0.0)
    assert (cov[0], cov[7], cov[35]) == (0.1, 0.2, 0.3)


def test_drifted_covariance_grows_quadratically_with_distance():
    """Std-dev grows linearly with distance, so variance grows quadratically."""
    base = [1e-3, 1e-3, 1e-3]
    drift = [0.005, 0.01, 0.02]
    cov = drifted_covariance(base, drift, 10.0)
    assert math.isclose(cov[0], 1e-3 + (0.005 * 10.0) ** 2)
    assert math.isclose(cov[7], 1e-3 + (0.01 * 10.0) ** 2)
    assert math.isclose(cov[35], 1e-3 + (0.02 * 10.0) ** 2)


def test_body_velocity_identity_at_zero_heading():
    """With the robot facing along odom x, body and odom frames coincide."""
    velocity = qwiic_otos.Pose2D(0.5, 0.2, 0.0)
    out = body_velocity_from_odom(velocity, 0.0)
    assert math.isclose(out.x, velocity.x, abs_tol=1e-9)
    assert math.isclose(out.y, velocity.y, abs_tol=1e-9)


def test_body_velocity_quarter_turn_puts_speed_in_x():
    """Facing odom +y, odom-frame (0, V) is forward motion: body (V, 0)."""
    velocity = qwiic_otos.Pose2D(0.0, 0.33, 0.0)
    out = body_velocity_from_odom(velocity, math.pi / 2.0)
    assert math.isclose(out.x, velocity.y, abs_tol=1e-9)
    assert math.isclose(out.y, -velocity.x, abs_tol=1e-9)


def test_body_velocity_does_not_alias_its_input():
    """The input velocity must be left untouched, or x leaks into y."""
    velocity = qwiic_otos.Pose2D(0.0, 0.33, 0.0)
    body_velocity_from_odom(velocity, math.pi / 2.0)
    assert (velocity.x, velocity.y) == (0.0, 0.33)


def test_body_velocity_preserves_yaw_rate():
    """A rotation about z does not change the yaw rate, but it must survive."""
    velocity = qwiic_otos.Pose2D(0.4, -0.1, 0.75)
    out = body_velocity_from_odom(velocity, 1.2)
    assert out.h == velocity.h
