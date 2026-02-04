#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU Kalman Filter Implementation using filterpy

This module implements a Kalman Filter for IMU data processing, including:
- State estimation for position, velocity, and orientation
- Noise reduction for accelerometer and gyroscope measurements
- Sensor fusion to combine accelerometer and gyroscope data

Author: LIOKit Project
Date: 2026-02-04
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
from scipy.spatial.transform import Rotation as R
from filterpy.kalman import KalmanFilter
from typing import Tuple, Optional, List
import warnings

warnings.filterwarnings('ignore')


class IMUKalmanFilter:
    """
    IMU-based Kalman Filter for state estimation

    State Vector (16-dimensional):
    - position: [x, y, z] (3)
    - velocity: [vx, vy, vz] (3)
    - orientation: [qx, qy, qz, qw] (4) - quaternion
    - acceleration_bias: [bx, by, bz] (3)
    - gyro_bias: [bgx, bgy, bgz] (3)

    Total: 3 + 3 + 4 + 3 + 3 = 16
    """

    def __init__(self, dt: float = 0.01, process_noise_pos: float = 1e-3, process_noise_vel: float = 1e-2, process_noise_acc_bias: float = 1e-4, process_noise_gyro_bias: float = 1e-4, meas_noise_acc: float = 0.1, meas_noise_gyro: float = 0.01):
        """
        Initialize IMU Kalman Filter

        Parameters:
        -----------
        dt : float
            Time step (seconds), default 0.01 (100 Hz)
        process_noise_pos : float
            Process noise for position
        process_noise_vel : float
            Process noise for velocity
        process_noise_acc_bias : float
            Process noise for accelerometer bias
        process_noise_gyro_bias : float
            Process noise for gyroscope bias
        meas_noise_acc : float
            Measurement noise for accelerometer
        meas_noise_gyro : float
            Measurement noise for gyroscope
        """
        self.dt = dt

        # State dimension
        self.dim_state = 16  # pos(3) + vel(3) + quat(4) + acc_bias(3) + gyro_bias(3)
        self.dim_meas = 6  # acc(3) + gyro(3)

        # Initialize Kalman Filter
        self.kf = KalmanFilter(dim_x=self.dim_state, dim_z=self.dim_meas)

        # State vector: [x, y, z, vx, vy, vz, qx, qy, qz, qw, bx, by, bz, bgx, bgy, bgz]
        self.kf.x = np.zeros(self.dim_state)
        self.kf.x[9] = 1.0  # Initialize quaternion qw = 1 (identity rotation)

        # State transition matrix F
        self._init_state_transition()

        # Control matrix B (maps acceleration to velocity/position)
        self._init_control_matrix()

        # Measurement matrix H
        self._init_measurement_matrix()

        # Process noise covariance Q
        self._init_process_noise(process_noise_pos, process_noise_vel, process_noise_acc_bias, process_noise_gyro_bias)

        # Measurement noise covariance R
        self._init_measurement_noise(meas_noise_acc, meas_noise_gyro)

        # Initial state covariance P
        self.kf.P = np.eye(self.dim_state) * 0.1

        # History for visualization
        self.history = {'position': [], 'velocity': [], 'orientation': [], 'acceleration': [], 'gyroscope': []}

    def _init_state_transition(self):
        """Initialize state transition matrix F"""
        F = np.eye(self.dim_state)

        # Position integration: pos_new = pos_old + vel * dt
        F[0, 3] = self.dt  # x += vx * dt
        F[1, 4] = self.dt  # y += vy * dt
        F[2, 5] = self.dt  # z += vz * dt

        # Acceleration bias (random walk)
        F[10, 10] = 1.0
        F[11, 11] = 1.0
        F[12, 12] = 1.0

        # Gyroscope bias (random walk)
        F[13, 13] = 1.0
        F[14, 14] = 1.0
        F[15, 15] = 1.0

        self.kf.F = F

    def _init_control_matrix(self):
        """Initialize control matrix B to map acceleration control input into state increments"""
        # Control vector u will be 3D accelerometer (ax, ay, az)
        B = np.zeros((self.dim_state, 3))

        # Position gets 0.5 * dt^2 * acc
        dt2 = 0.5 * (self.dt ** 2)
        B[0, 0] = dt2
        B[1, 1] = dt2
        B[2, 2] = dt2

        # Velocity gets dt * acc
        B[3, 0] = self.dt
        B[4, 1] = self.dt
        B[5, 2] = self.dt

        # No direct control on orientation or biases here
        self.kf.B = B

    def _init_measurement_matrix(self):
        """Initialize measurement matrix H"""
        H = np.zeros((self.dim_meas, self.dim_state))

        # We measure acceleration directly (minus bias)
        # Note: In practice, we need to account for gravity and orientation
        H[0, 10] = 1.0  # acc_x = true_acc_x + bias_x
        H[1, 11] = 1.0  # acc_y = true_acc_y + bias_y
        H[2, 12] = 1.0  # acc_z = true_acc_z + bias_z

        # We measure gyroscope directly (minus bias)
        H[3, 13] = 1.0  # gyro_x = true_gyro_x + gyro_bias_x
        H[4, 14] = 1.0  # gyro_y = true_gyro_y + gyro_bias_y
        H[5, 15] = 1.0  # gyro_z = true_gyro_z + gyro_bias_z

        self.kf.H = H

    def _init_process_noise(self, noise_pos, noise_vel, noise_acc_bias, noise_gyro_bias):
        """Initialize process noise covariance Q"""
        Q = np.eye(self.dim_state)

        # Position noise
        Q[0, 0] = noise_pos
        Q[1, 1] = noise_pos
        Q[2, 2] = noise_pos

        # Velocity noise
        Q[3, 3] = noise_vel
        Q[4, 4] = noise_vel
        Q[5, 5] = noise_vel

        # Orientation noise (quaternion)
        Q[6, 6] = noise_vel * 0.1
        Q[7, 7] = noise_vel * 0.1
        Q[8, 8] = noise_vel * 0.1
        Q[9, 9] = noise_vel * 0.1

        # Acceleration bias noise
        Q[10, 10] = noise_acc_bias
        Q[11, 11] = noise_acc_bias
        Q[12, 12] = noise_acc_bias

        # Gyroscope bias noise
        Q[13, 13] = noise_gyro_bias
        Q[14, 14] = noise_gyro_bias
        Q[15, 15] = noise_gyro_bias

        self.kf.Q = Q

    def _init_measurement_noise(self, meas_noise_acc, meas_noise_gyro):
        """Initialize measurement noise covariance R"""
        R = np.eye(self.dim_meas)

        # Accelerometer measurement noise
        R[0, 0] = meas_noise_acc
        R[1, 1] = meas_noise_acc
        R[2, 2] = meas_noise_acc

        # Gyroscope measurement noise
        R[3, 3] = meas_noise_gyro
        R[4, 4] = meas_noise_gyro
        R[5, 5] = meas_noise_gyro

        self.kf.R = R

    def predict(self, acc: np.ndarray, gyro: np.ndarray) -> np.ndarray:
        """
        Predict step using IMU measurements

        Parameters:
        -----------
        acc : np.ndarray
            Acceleration measurement [ax, ay, az] (m/s²)
        gyro : np.ndarray
            Angular velocity measurement [wx, wy, wz] (rad/s)

        Returns:
        --------
        np.ndarray
            Predicted state vector
        """
        # Compensate for bias
        acc_corrected = acc - self.kf.x[10:13]
        gyro_corrected = gyro - self.kf.x[13:16]

        # Update orientation using gyroscope (integrate angular velocity)
        current_quat = self.kf.x[6:10]
        rot = R.from_quat(current_quat)

        # Create rotation from angular velocity vector directly
        rotvec = gyro_corrected * self.dt
        if np.linalg.norm(rotvec) > 1e-12:
            delta_rot = R.from_rotvec(rotvec)
            new_rot = rot * delta_rot
            q = new_rot.as_quat()
            # Normalize quaternion
            nrm = np.linalg.norm(q)
            if nrm > 0:
                self.kf.x[6:10] = q / nrm
            else:
                self.kf.x[6:10] = q

        # Use acceleration as control input (B matrix maps acc -> state increments)
        # Call predict with control to update state and covariance consistently
        try:
            self.kf.predict(u=acc_corrected)
        except TypeError:
            # Older filterpy versions may not accept u named; try positional
            self.kf.predict(acc_corrected)

        return self.kf.x

    def update(self, acc: np.ndarray, gyro: np.ndarray) -> np.ndarray:
        """
        Update step using IMU measurements

        Parameters:
        -----------
        acc : np.ndarray
            Acceleration measurement [ax, ay, az] (m/s²)
        gyro : np.ndarray
            Angular velocity measurement [wx, wy, wz] (rad/s)

        Returns:
        --------
        np.ndarray
            Updated state vector
        """
        # Measurement vector
        z = np.concatenate([acc, gyro])

        # Update Kalman filter
        self.kf.update(z)

        # Normalize quaternion to ensure unit norm
        self.kf.x[6:10] /= np.linalg.norm(self.kf.x[6:10])

        return self.kf.x

    def filter_batch(self, acc_data: np.ndarray, gyro_data: np.ndarray, timestamps: np.ndarray) -> dict:
        """
        Filter a batch of IMU data

        Parameters:
        -----------
        acc_data : np.ndarray
            Acceleration data (N x 3)
        gyro_data : np.ndarray
            Angular velocity data (N x 3)
        timestamps : np.ndarray
            Timestamps for each measurement

        Returns:
        --------
        dict
            Filtered data including position, velocity, orientation
        """
        N = len(acc_data)

        filtered_position = np.zeros((N, 3))
        filtered_velocity = np.zeros((N, 3))
        filtered_orientation = np.zeros((N, 4))
        filtered_acc = np.zeros((N, 3))
        filtered_gyro = np.zeros((N, 3))

        for i in range(N):
            # Predict
            self.predict(acc_data[i], gyro_data[i])

            # Update
            self.update(acc_data[i], gyro_data[i])

            # Store results
            filtered_position[i] = self.kf.x[0:3]
            filtered_velocity[i] = self.kf.x[3:6]
            filtered_orientation[i] = self.kf.x[6:10]

            # Extract compensated measurements
            filtered_acc[i] = acc_data[i] - self.kf.x[10:13]
            filtered_gyro[i] = gyro_data[i] - self.kf.x[13:16]

        return {'position': filtered_position, 'velocity': filtered_velocity, 'orientation': filtered_orientation, 'acceleration': filtered_acc, 'gyroscope': filtered_gyro, 'acc_bias': self.kf.x[10:13], 'gyro_bias': self.kf.x[13:16]}

    def get_state(self) -> dict:
        """
        Get current filter state

        Returns:
        --------
        dict
            Current state including position, velocity, orientation, and biases
        """
        return {'position': self.kf.x[0:3], 'velocity': self.kf.x[3:6], 'orientation': self.kf.x[6:10], 'acc_bias': self.kf.x[10:13], 'gyro_bias': self.kf.x[13:16]}


def compute_noise_reduction_metrics(raw_data: np.ndarray, filtered_data: np.ndarray) -> dict:
    """
    Compute noise reduction metrics

    Parameters:
    -----------
    raw_data : np.ndarray
        Raw data
    filtered_data : np.ndarray
        Filtered data

    Returns:
    --------
    dict
        Noise reduction metrics including std reduction, smoothness improvement
    """
    metrics = {}

    # Standard deviation reduction
    raw_std = np.std(raw_data, axis=0)
    filtered_std = np.std(filtered_data, axis=0)
    metrics['std_raw'] = raw_std
    metrics['std_filtered'] = filtered_std
    # avoid division by zero
    metrics['std_reduction_pct'] = np.where(raw_std != 0, (raw_std - filtered_std) / raw_std * 100, 0.0)

    # Smoothness (jerk/acceleration of changes)
    raw_changes = np.diff(raw_data, axis=0)
    filtered_changes = np.diff(filtered_data, axis=0)
    raw_smoothness = np.std(raw_changes, axis=0)
    filtered_smoothness = np.std(filtered_changes, axis=0)
    metrics['smoothness_raw'] = raw_smoothness
    metrics['smoothness_filtered'] = filtered_smoothness
    metrics['smoothness_improvement_pct'] = np.where(raw_smoothness != 0, (raw_smoothness - filtered_smoothness) / raw_smoothness * 100, 0.0)

    return metrics


def integrate_imu_data(acc_world: np.ndarray, gyro: np.ndarray, dt: np.ndarray, timestamps: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Integrate IMU data to compute position and velocity

    Parameters:
    -----------
    acc_world : np.ndarray
        Acceleration in world coordinate system (N x 3)
    gyro : np.ndarray
        Angular velocity (N x 3)
    dt : np.ndarray
        Time intervals (N,)
    timestamps : np.ndarray
        Timestamps (N,)

    Returns:
    --------
    Tuple[np.ndarray, np.ndarray, np.ndarray]
        position: Position (N x 3)
        velocity: Velocity (N x 3)
        orientation: Orientation quaternions (N x 4)
    """
    N = len(acc_world)

    # Initialize
    position = np.zeros((N, 3))
    velocity = np.zeros((N, 3))
    orientation = np.zeros((N, 4))

    # Initial state
    vel = np.zeros(3)
    pos = np.zeros(3)
    current_rot = R.identity()
    orientation[0] = current_rot.as_quat()

    for i in range(1, N):
        # Get current time step
        dt_i = dt[i]
        if dt_i <= 0:
            # Keep previous state
            position[i] = position[i - 1]
            velocity[i] = velocity[i - 1]
            orientation[i] = orientation[i - 1]
            continue

        # 1. Update orientation using angular velocity integration
        omega = gyro[i]
        rotvec = omega * dt_i

        if np.linalg.norm(rotvec) > 1e-12:
            delta_rot = R.from_rotvec(rotvec)
            current_rot = current_rot * delta_rot

        # Save current orientation
        orientation[i] = current_rot.as_quat()

        # 2. Update velocity and position using world coordinate acceleration
        acc = acc_world[i]
        vel += acc * dt_i
        pos += vel * dt_i

        # Save results
        velocity[i] = vel
        position[i] = pos

    return position, velocity, orientation


# Example usage
if __name__ == "__main__":
    print('')
