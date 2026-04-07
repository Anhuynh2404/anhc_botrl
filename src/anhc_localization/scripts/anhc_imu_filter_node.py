#!/usr/bin/env python3
"""IMU complementary filter node.

Fuses accelerometer (roll/pitch estimation) with gyroscope integration
using a simple complementary filter — no external library required.
Publishes a smoothed Imu message on /imu/data_filtered.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class AhncImuFilterNode(Node):

    def __init__(self):
        super().__init__('anhc_imu_filter_node')

        # Complementary filter coefficient (0 = pure accel, 1 = pure gyro)
        self.declare_parameter('alpha', 0.98)
        # Low-pass smoothing for linear acceleration
        self.declare_parameter('accel_lpf_alpha', 0.8)

        self._alpha = self.get_parameter('alpha').value
        self._accel_alpha = self.get_parameter('accel_lpf_alpha').value

        # Filter state
        self._roll = 0.0
        self._pitch = 0.0
        self._last_stamp = None
        self._accel_filt = [0.0, 0.0, 9.81]  # smoothed accel

        self._sub = self.create_subscription(
            Imu, '/imu/data', self._imu_cb, 10)
        self._pub = self.create_publisher(Imu, '/imu/data_filtered', 10)

        self.get_logger().info('anhc_imu_filter_node started')

    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu) -> None:
        stamp = msg.header.stamp
        now_sec = stamp.sec + stamp.nanosec * 1e-9

        if self._last_stamp is None:
            self._last_stamp = now_sec
            return

        dt = now_sec - self._last_stamp
        self._last_stamp = now_sec

        if dt <= 0.0 or dt > 1.0:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y

        # Low-pass filter on linear acceleration
        ka = self._accel_alpha
        self._accel_filt[0] = ka * self._accel_filt[0] + (1.0 - ka) * ax
        self._accel_filt[1] = ka * self._accel_filt[1] + (1.0 - ka) * ay
        self._accel_filt[2] = ka * self._accel_filt[2] + (1.0 - ka) * az
        fax, fay, faz = self._accel_filt

        # Accelerometer-derived roll and pitch
        accel_norm = math.sqrt(fax ** 2 + fay ** 2 + faz ** 2)
        if accel_norm > 0.1:
            accel_roll = math.atan2(fay, math.sqrt(fax ** 2 + faz ** 2))
            accel_pitch = math.atan2(-fax, math.sqrt(fay ** 2 + faz ** 2))
        else:
            accel_roll = self._roll
            accel_pitch = self._pitch

        # Complementary fusion
        alpha = self._alpha
        self._roll = alpha * (self._roll + gx * dt) + (1.0 - alpha) * accel_roll
        self._pitch = alpha * (self._pitch + gy * dt) + (1.0 - alpha) * accel_pitch

        # Roll/pitch → quaternion (yaw assumed 0 here; EKF handles yaw)
        cr = math.cos(self._roll * 0.5)
        sr = math.sin(self._roll * 0.5)
        cp = math.cos(self._pitch * 0.5)
        sp = math.sin(self._pitch * 0.5)

        out = Imu()
        out.header = msg.header

        out.orientation.w = cr * cp
        out.orientation.x = sr * cp
        out.orientation.y = cr * sp
        out.orientation.z = -sr * sp

        out.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.10,   # yaw uncertain
        ]

        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance

        out.linear_acceleration.x = fax
        out.linear_acceleration.y = fay
        out.linear_acceleration.z = faz
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AhncImuFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
