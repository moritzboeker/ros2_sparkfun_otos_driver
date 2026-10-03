# Copyright 2026 Moritz Emanuel Boeker
# SPDX-License-Identifier: MIT
"""
ROS 2 driver for the SparkFun Optical Tracking Odometry Sensor (OTOS).

Reads pose and velocity from the PAA5160E1-based OTOS board over I2C via
the sparkfun-qwiic-otos library and publishes them as nav_msgs/Odometry,
optionally broadcasting the corresponding odom -> base_link transform.
"""

import math

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import qwiic_otos
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# Variance assigned to the dimensions the sensor cannot measure
# (z, roll, pitch), so downstream consumers ignore them.
UNMEASURED_VARIANCE = 1e6

# Consecutive read failures before attempting to re-initialize the sensor.
MAX_READ_FAILURES = 5


def quaternion_from_yaw(yaw):
    """Return a geometry_msgs Quaternion for a rotation about z only."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def diagonal_covariance(x_var, y_var, yaw_var):
    """Return a 6x6 row-major covariance with the given planar variances."""
    cov = [0.0] * 36
    cov[0] = x_var                    # x
    cov[7] = y_var                    # y
    cov[14] = UNMEASURED_VARIANCE     # z
    cov[21] = UNMEASURED_VARIANCE     # roll
    cov[28] = UNMEASURED_VARIANCE     # pitch
    cov[35] = yaw_var                 # yaw
    return cov


def drifted_covariance(base_variance, drift_per_meter, distance):
    """
    Return a planar covariance whose error grows with distance traveled.

    Odometry drift is dominated by systematic scale error, so the standard
    deviation (not the variance) grows linearly with distance:
    var_i = base_i + (drift_i * distance)^2.
    """
    return diagonal_covariance(
        *(base + (drift * distance) ** 2
          for base, drift in zip(base_variance, drift_per_meter)))


def body_velocity_from_odom(velocity, yaw):
    """
    Express an odom-frame velocity in the robot body frame.

    The OTOS reports velocity in the frame it tracks in, which is fixed at
    resetTracking(), while nav_msgs/Odometry defines twist in child_frame_id
    (base_link here). Re-expressing a vector in axes that are themselves
    rotated by +yaw transforms its components by the inverse rotation, R(-yaw):

        | cos(yaw)  sin(yaw)|
        |-sin(yaw)  cos(yaw)|

    The velocity itself is unchanged; only the axes it is measured against
    rotate. A rotation about z does not alter the yaw rate, so it passes
    through untouched.
    """
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return qwiic_otos.Pose2D(
        cos_yaw * velocity.x + sin_yaw * velocity.y,
        -sin_yaw * velocity.x + cos_yaw * velocity.y,
        velocity.h)


class OtosNode(Node):
    """Publish odometry from the SparkFun OTOS and offer runtime services."""

    def __init__(self):
        super().__init__('otos')
        self._declare_parameters()

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.pose_variance = self.get_parameter('pose_variance').value
        self.pose_drift = self.get_parameter('pose_drift_per_meter').value
        self.twist_covariance = diagonal_covariance(
            *self.get_parameter('twist_variance').value)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_service(Trigger, '~/reset_tracking', self.on_reset_tracking)
        self.create_service(Trigger, '~/calibrate_imu', self.on_calibrate_imu)

        self.sensor = qwiic_otos.QwiicOTOS()
        self.is_ready = False
        self.read_failures = 0
        self.last_pose = qwiic_otos.Pose2D()
        self.distance_traveled = 0.0

        publish_rate = self.get_parameter('publish_rate').value
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.on_timer)
        self.init_timer = self.create_timer(2.0, self.on_init_timer)
        self.on_init_timer()  # try immediately instead of waiting one period

    def _declare_parameters(self):
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('linear_scalar', 1.0)
        self.declare_parameter('angular_scalar', 1.0)
        self.declare_parameter('offset.x', 0.0)
        self.declare_parameter('offset.y', 0.0)
        self.declare_parameter('offset.yaw', 0.0)
        # Planar variances [x, y, yaw]. The OTOS reports its own standard
        # deviations, but SparkFun states they do not represent actual error,
        # so user-tunable values are published instead. The pose variance
        # grows with distance traveled (see drifted_covariance); the drift
        # default matches SparkFun's typical accuracy of 0.5 % of distance.
        self.declare_parameter('pose_variance', [1e-3, 1e-3, 1e-3])
        self.declare_parameter('pose_drift_per_meter', [5e-3, 5e-3, 5e-3])
        self.declare_parameter('twist_variance', [1e-3, 1e-3, 1e-3])

    def on_init_timer(self):
        """Attempt sensor initialization until it succeeds."""
        if self.is_ready:
            return
        try:
            self._init_sensor()
        except OSError as e:
            self.get_logger().warn(
                f'OTOS initialization failed ({e}), retrying...',
                throttle_duration_sec=10.0)

    def _init_sensor(self):
        if not self.sensor.is_connected():
            self.get_logger().warn(
                'OTOS not found on the I2C bus, retrying...',
                throttle_duration_sec=10.0)
            return
        if not self.sensor.begin():
            self.get_logger().warn(
                'OTOS begin() failed, retrying...', throttle_duration_sec=10.0)
            return

        self.sensor.setLinearUnit(self.sensor.kLinearUnitMeters)
        self.sensor.setAngularUnit(self.sensor.kAngularUnitRadians)

        linear_scalar = self.get_parameter('linear_scalar').value
        angular_scalar = self.get_parameter('angular_scalar').value
        if not self.sensor.setLinearScalar(linear_scalar):
            self.get_logger().error(
                f'linear_scalar {linear_scalar} out of range '
                f'[{self.sensor.kMinScalar}, {self.sensor.kMaxScalar}], using 1.0')
        if not self.sensor.setAngularScalar(angular_scalar):
            self.get_logger().error(
                f'angular_scalar {angular_scalar} out of range '
                f'[{self.sensor.kMinScalar}, {self.sensor.kMaxScalar}], using 1.0')

        offset = qwiic_otos.Pose2D(
            self.get_parameter('offset.x').value,
            self.get_parameter('offset.y').value,
            self.get_parameter('offset.yaw').value)
        self.sensor.setOffset(offset)

        self.get_logger().info('Calibrating IMU, keep the robot still...')
        self.sensor.calibrateImu()
        self.sensor.resetTracking()
        # After a mid-run re-initialization, continue from the last known
        # pose instead of silently jumping back to zero.
        if (self.last_pose.x, self.last_pose.y, self.last_pose.h) != (0.0, 0.0, 0.0):
            self.sensor.setPosition(self.last_pose)

        self.read_failures = 0
        self.is_ready = True
        self.get_logger().info('OTOS is ready')

    def on_timer(self):
        """Read the sensor and publish odometry (and TF, if enabled)."""
        if not self.is_ready:
            return
        try:
            pose = self.sensor.getPosition()
            vel = self.sensor.getVelocity()
        except OSError as e:
            self.read_failures += 1
            self.get_logger().error(
                f'OTOS read failed ({e})', throttle_duration_sec=5.0)
            if self.read_failures >= MAX_READ_FAILURES:
                self.get_logger().error(
                    'Too many consecutive read failures, re-initializing OTOS')
                self.is_ready = False
            return

        self.read_failures = 0
        self.distance_traveled += math.hypot(
            pose.x - self.last_pose.x, pose.y - self.last_pose.y)
        self.last_pose = pose
        stamp = self.get_clock().now().to_msg()
        self.publish_odom(stamp, pose, vel)
        if self.tf_broadcaster is not None:
            self.broadcast_tf(stamp, pose)

    def publish_odom(self, stamp, pose, vel):
        """Publish a nav_msgs/Odometry message for the given readings."""
        vel_body = body_velocity_from_odom(vel, pose.h)
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = pose.x
        msg.pose.pose.position.y = pose.y
        msg.pose.pose.orientation = quaternion_from_yaw(pose.h)
        msg.pose.covariance = drifted_covariance(
            self.pose_variance, self.pose_drift, self.distance_traveled)
        msg.twist.twist.linear.x = vel_body.x
        msg.twist.twist.linear.y = vel_body.y
        msg.twist.twist.angular.z = vel_body.h
        msg.twist.covariance = self.twist_covariance
        self.odom_pub.publish(msg)

    def broadcast_tf(self, stamp, pose):
        """Broadcast the frame_id -> child_frame_id transform."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = pose.x
        t.transform.translation.y = pose.y
        t.transform.rotation = quaternion_from_yaw(pose.h)
        self.tf_broadcaster.sendTransform(t)

    def on_reset_tracking(self, request, response):
        """Zero the sensor's pose estimate."""
        if not self.is_ready:
            response.success = False
            response.message = 'OTOS is not initialized'
            return response
        try:
            self.sensor.resetTracking()
            self.last_pose = qwiic_otos.Pose2D()
            self.distance_traveled = 0.0
            response.success = True
            response.message = 'Tracking reset to origin'
        except OSError as e:
            response.success = False
            response.message = f'Reset failed: {e}'
        return response

    def on_calibrate_imu(self, request, response):
        """Recalibrate the IMU; the robot must be stationary."""
        if not self.is_ready:
            response.success = False
            response.message = 'OTOS is not initialized'
            return response
        try:
            self.get_logger().info('Calibrating IMU, keep the robot still...')
            ok = self.sensor.calibrateImu()
            response.success = bool(ok)
            response.message = 'IMU calibrated' if ok else 'IMU calibration failed'
        except OSError as e:
            response.success = False
            response.message = f'Calibration failed: {e}'
        return response


def main(args=None):
    """Run the OTOS driver node."""
    rclpy.init(args=args)
    node = OtosNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
