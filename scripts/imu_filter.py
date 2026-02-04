import numpy as np
from scipy.signal import butter, filtfilt
from scipy.spatial.transform import Rotation as R
from typing import Optional, Tuple, Dict, Any


def _lowpass_filter(data: np.ndarray, fs: float, cutoff: float = 5.0, order: int = 4) -> np.ndarray:
    """Apply Butterworth low-pass filter to data.

    Args:
        data: Input data array (N x M) where N is samples, M is channels
        fs: Sampling frequency in Hz
        cutoff: Cutoff frequency in Hz
        order: Filter order

    Returns:
        Filtered data array

    Raises:
        ValueError: If fs <= 0 or cutoff is invalid
    """
    if fs <= 0:
        raise ValueError("Sampling frequency must be positive")
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    if not 0 < normal_cutoff < 1:
        raise ValueError("Cutoff frequency must be between 0 and Nyquist frequency")
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data, axis=0)


def integrate_imu_data(acc: np.ndarray, gyro: np.ndarray, dt: float, initial_pos: Optional[np.ndarray] = None, initial_vel: Optional[np.ndarray] = None, initial_quat: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate IMU data to compute position, velocity, and orientation.

    This function performs numerical integration of accelerometer and gyroscope
    data to estimate position, velocity, and orientation over time.

    Args:
        acc: Acceleration data in body frame (N x 3) in m/s²
        gyro: Angular velocity data in body frame (N x 3) in rad/s
        dt: Time step in seconds
        initial_pos: Initial position (3,), defaults to [0, 0, 0]
        initial_vel: Initial velocity (3,), defaults to [0, 0, 0]
        initial_quat: Initial quaternion (4,) in [x, y, z, w] format, defaults to identity

    Returns:
        Tuple of (position, velocity, orientation):
            - position: Position in world frame (N x 3) in meters
            - velocity: Velocity in world frame (N x 3) in m/s
            - orientation: Orientation as quaternions (N x 4) in [x, y, z, w] format

    Raises:
        ValueError: If input shapes are invalid or dt <= 0
    """
    acc = np.asarray(acc)
    gyro = np.asarray(gyro)

    if acc.shape != gyro.shape:
        raise ValueError(f"acc and gyro must have same shape, got {acc.shape} and {gyro.shape}")
    if acc.ndim != 2 or acc.shape[1] != 3:
        raise ValueError(f"acc and gyro must be (N, 3) arrays, got shape {acc.shape}")
    if dt <= 0:
        raise ValueError(f"dt must be positive, got {dt}")

    N = len(acc)

    # Initialize arrays
    position = np.zeros((N, 3))
    velocity = np.zeros((N, 3))
    orientation = np.zeros((N, 4))

    # Set initial conditions
    vel = np.zeros(3) if initial_vel is None else np.asarray(initial_vel, dtype=float)
    pos = np.zeros(3) if initial_pos is None else np.asarray(initial_pos, dtype=float)

    # Initial orientation (identity quaternion [0, 0, 0, 1])
    current_rot = R.identity()
    if initial_quat is not None:
        current_rot = R.from_quat(initial_quat)

    orientation[0] = current_rot.as_quat()
    velocity[0] = vel
    position[0] = pos

    # Integrate
    for i in range(1, N):
        # Update orientation using gyroscope data
        omega = gyro[i]
        angle = np.linalg.norm(omega) * dt

        if angle > 1e-10:  # Avoid division by zero
            axis = omega / np.linalg.norm(omega)
            delta_rot = R.from_rotvec(axis * angle)
            current_rot = current_rot * delta_rot

        orientation[i] = current_rot.as_quat()

        # Transform acceleration to world frame
        rotation_matrix = current_rot.as_matrix()
        acc_world = rotation_matrix @ acc[i]

        # Update velocity and position
        vel += acc_world * dt
        pos += vel * dt

        velocity[i] = vel
        position[i] = pos

    return position, velocity, orientation


class IMUFilter:
    """IMU data filter using Butterworth low-pass filter.

    This class provides filtering capabilities for IMU (Inertial Measurement Unit)
    data, including accelerometer and gyroscope measurements. It uses a Butterworth
    low-pass filter to reduce noise while preserving signal characteristics.

    Attributes:
        dt: Time step in seconds
        fs: Sampling frequency in Hz
        cutoff: Cutoff frequency in Hz
        order: Filter order
    """

    def __init__(self, dt: float = None, cutoff_hz: float = 5.0, order: int = 4, **kwargs):
        """Initialize IMU filter.

        Args:
            dt: Time step in seconds (required)
            cutoff_hz: Cutoff frequency in Hz (default: 5.0)
            order: Filter order (default: 4)
            **kwargs: Legacy keyword arguments (only 'dt', 'cutoff_hz', 'order' are used)

        Raises:
            ValueError: If dt is None or dt <= 0
        """
        # Accept legacy kwargs but only use dt / cutoff / order
        if dt is None:
            dt = kwargs.get('dt', None)
        if dt is None:
            raise ValueError('dt must be provided')

        self.dt = float(dt)
        if self.dt <= 0:
            raise ValueError('dt must be positive')

        self.fs = 1.0 / self.dt
        self.cutoff = float(kwargs.get('cutoff_hz', cutoff_hz))
        self.order = int(kwargs.get('order', order))

        # Validate cutoff frequency
        nyq = 0.5 * self.fs
        if self.cutoff <= 0 or self.cutoff >= nyq:
            raise ValueError(f'cutoff_hz must be between 0 and Nyquist frequency ({nyq:.2f} Hz)')

    def filter(self, acc: np.ndarray, gyro: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Apply low-pass filter to accelerometer and gyroscope data.

        Args:
            acc: Accelerometer data (N x 3) in m/s²
            gyro: Gyroscope data (N x 3) in rad/s

        Returns:
            Tuple of (filtered_acc, filtered_gyro)
        """
        acc = np.asarray(acc)
        gyro = np.asarray(gyro)

        if acc.shape != gyro.shape:
            raise ValueError(f"acc and gyro must have same shape, got {acc.shape} and {gyro.shape}")

        acc_f = _lowpass_filter(acc, self.fs, self.cutoff, self.order)
        gyro_f = _lowpass_filter(gyro, self.fs, self.cutoff, self.order)

        return acc_f, gyro_f

    def filter_batch(self, acc: np.ndarray, gyro: np.ndarray, timestamps: Optional[np.ndarray] = None, initial_pos: Optional[np.ndarray] = None, initial_vel: Optional[np.ndarray] = None, initial_quat: Optional[np.ndarray] = None) -> Dict[str, np.ndarray]:
        """Apply filtering and perform integration to produce filtered states.

        Args:
            acc: Accelerometer data (N x 3) in m/s²
            gyro: Gyroscope data (N x 3) in rad/s
            timestamps: Optional timestamps array (N,), used to compute dt if provided
            initial_pos: Initial position (3,), defaults to [0, 0, 0]
            initial_vel: Initial velocity (3,), defaults to [0, 0, 0]
            initial_quat: Initial quaternion (4,) in [x, y, z, w] format, defaults to identity

        Returns:
            Dictionary with keys:
                - 'position': Position in world frame (N x 3) in meters
                - 'velocity': Velocity in world frame (N x 3) in m/s
                - 'orientation': Orientation as quaternions (N x 4) in [x, y, z, w] format
                - 'acceleration': Filtered acceleration (N x 3) in m/s²
                - 'gyroscope': Filtered gyroscope (N x 3) in rad/s
        """
        acc = np.asarray(acc)
        gyro = np.asarray(gyro)

        # Compute sampling dt if timestamps provided
        dt_use = self.dt
        if timestamps is not None:
            ts = np.asarray(timestamps)
            if len(ts) >= 2:
                diffs = np.diff(ts)
                diffs = diffs[diffs > 0]
                if diffs.size > 0:
                    dt_use = float(np.mean(diffs))

        acc_f, gyro_f = self.filter(acc, gyro)
        pos_f, vel_f, ori_f = integrate_imu_data(acc_f, gyro_f, dt_use, initial_pos=initial_pos, initial_vel=initial_vel, initial_quat=initial_quat)

        return {
            'position': pos_f,
            'velocity': vel_f,
            'orientation': ori_f,
            'acceleration': acc_f,
            'gyroscope': gyro_f,
        }


def compute_noise_reduction_metrics(raw: np.ndarray, filtered: np.ndarray) -> Dict[str, np.ndarray]:
    """Compute noise reduction metrics between raw and filtered data.

    Args:
        raw: Raw data array (N x M)
        filtered: Filtered data array (N x M)

    Returns:
        Dictionary with keys:
            - 'rmse': Root mean square error (M,)
            - 'std_raw': Standard deviation of raw data (M,)
            - 'std_filtered': Standard deviation of filtered data (M,)
            - 'std_reduction_pct': Percentage reduction in std (M,)
            - 'smoothness_improvement_pct': Percentage improvement in smoothness (M,)

    Raises:
        ValueError: If raw and filtered have different shapes
    """
    raw = np.asarray(raw)
    filtered = np.asarray(filtered)

    if raw.shape != filtered.shape:
        raise ValueError(f'raw and filtered must have the same shape, got {raw.shape} and {filtered.shape}')

    diff = raw - filtered
    rmse = np.sqrt(np.mean(diff**2, axis=0))
    std_raw = np.std(raw, axis=0)
    std_f = np.std(filtered, axis=0)

    # Percent reduction (handle zero safely)
    with np.errstate(divide='ignore', invalid='ignore'):
        std_reduction_pct = np.where(std_raw == 0, 0.0, (std_raw - std_f) / std_raw * 100.0)

    # Smoothness measured via mean absolute derivative
    smooth_raw = np.mean(np.abs(np.diff(raw, axis=0)), axis=0)
    smooth_f = np.mean(np.abs(np.diff(filtered, axis=0)), axis=0)
    with np.errstate(divide='ignore', invalid='ignore'):
        smoothness_improvement_pct = np.where(smooth_raw == 0, 0.0, (smooth_raw - smooth_f) / smooth_raw * 100.0)

    return {
        'rmse': rmse,
        'std_raw': std_raw,
        'std_filtered': std_f,
        'std_reduction_pct': std_reduction_pct,
        'smoothness_improvement_pct': smoothness_improvement_pct,
    }

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # Generate test data for _lowpass_filter
    print("Testing _lowpass_filter function...")

    # Parameters
    fs = 100.0  # Sampling frequency (Hz)
    duration = 10.0  # Duration (seconds)
    cutoff = 5.0  # Cutoff frequency (Hz)
    order = 4  # Filter order

    # Generate time array
    t = np.arange(0, duration, 1/fs)
    N = len(t)

    # Generate synthetic signal: combination of low-frequency and high-frequency components
    # Low-frequency signal (should pass through filter)
    signal_low = 2.0 * np.sin(2 * np.pi * 1.0 * t)  # 1 Hz
    # High-frequency noise (should be filtered out)
    signal_high = 0.5 * np.sin(2 * np.pi * 20.0 * t)  # 20 Hz
    # Add random noise
    noise = 0.3 * np.random.randn(N)

    # Combine signals
    raw_signal = signal_low + signal_high + noise

    # Apply low-pass filter
    try:
        filtered_signal = _lowpass_filter(raw_signal.reshape(-1, 1), fs, cutoff, order).flatten()

        # Compute noise reduction metrics
        metrics = compute_noise_reduction_metrics(raw_signal.reshape(-1, 1), filtered_signal.reshape(-1, 1))

        print(f"\nFilter Parameters:")
        print(f"  Sampling frequency: {fs} Hz")
        print(f"  Cutoff frequency: {cutoff} Hz")
        print(f"  Filter order: {order}")
        print(f"  Duration: {duration} s")
        print(f"  Number of samples: {N}")

        print(f"\nNoise Reduction Metrics:")
        print(f"  RMSE: {metrics['rmse'][0]:.4f}")
        print(f"  Std (raw): {metrics['std_raw'][0]:.4f}")
        print(f"  Std (filtered): {metrics['std_filtered'][0]:.4f}")
        print(f"  Std reduction: {metrics['std_reduction_pct'][0]:.2f}%")
        print(f"  Smoothness improvement: {metrics['smoothness_improvement_pct'][0]:.2f}%")

        # Create visualization
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))

        # Plot 1: Raw vs Filtered signal
        axes[0].plot(t, raw_signal, 'b-', alpha=0.6, linewidth=0.8, label='Raw Signal')
        axes[0].plot(t, filtered_signal, 'r-', linewidth=1.5, label='Filtered Signal')
        axes[0].plot(t, signal_low, 'g--', alpha=0.5, linewidth=1.0, label='True Low-Freq Signal')
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Amplitude')
        axes[0].set_title('Low-Pass Filter: Raw vs Filtered Signal')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Plot 2: Zoomed view (first 2 seconds)
        zoom_idx = int(2 * fs)
        axes[1].plot(t[:zoom_idx], raw_signal[:zoom_idx], 'b-', alpha=0.6, linewidth=0.8, label='Raw Signal')
        axes[1].plot(t[:zoom_idx], filtered_signal[:zoom_idx], 'r-', linewidth=1.5, label='Filtered Signal')
        axes[1].plot(t[:zoom_idx], signal_low[:zoom_idx], 'g--', alpha=0.5, linewidth=1.0, label='True Low-Freq Signal')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Amplitude')
        axes[1].set_title('Zoomed View (First 2 Seconds)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Plot 3: Frequency spectrum
        from scipy.fft import fft, fftfreq

        fft_raw = np.abs(fft(raw_signal))
        fft_filtered = np.abs(fft(filtered_signal))
        freqs = fftfreq(N, 1/fs)

        # Plot only positive frequencies
        pos_freqs = freqs[:N//2]
        axes[2].semilogy(pos_freqs, fft_raw[:N//2], 'b-', alpha=0.6, linewidth=0.8, label='Raw Signal')
        axes[2].semilogy(pos_freqs, fft_filtered[:N//2], 'r-', linewidth=1.5, label='Filtered Signal')
        axes[2].axvline(cutoff, color='k', linestyle='--', alpha=0.7, label=f'Cutoff ({cutoff} Hz)')
        axes[2].set_xlabel('Frequency (Hz)')
        axes[2].set_ylabel('Magnitude')
        axes[2].set_title('Frequency Spectrum')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        axes[2].set_xlim(0, 30)  # Show up to 30 Hz

        plt.tight_layout()
        plt.savefig('scripts/lowpass_filter_test.png', dpi=150, bbox_inches='tight')
        print(f"\nVisualization saved to: scripts/lowpass_filter_test.png")
        plt.show()

        print("\nTest completed successfully!")

    except Exception as e:
        print(f"Error during test: {e}")
        import traceback
        traceback.print_exc()
