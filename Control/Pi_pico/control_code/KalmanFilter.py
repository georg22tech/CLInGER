import numpy as np


class VerticalKalmanFilter:
    def __init__(self, dt, imu_error, sensor_errors, initial_distances):
        self.dt = dt
        # Initialize state vector as 4x1 numpy array: pos1, vel1, pos2, vel2
        self.x = np.array([[0], [0], [-0.8], [0]], dtype=float)
        
        self.P = np.eye(4) * 0.01
        self.A = np.array([
            [1, dt, 0, 0],
            [0, 1,  0, 0],
            [0, 0,  1, dt],
            [0, 0,  0, 1]
        ])
        self.B = np.array([
            [0.5*dt**2, 0],
            [dt,        0],
            [0, 0.5*dt**2],
            [0, dt]
        ])
        imu_var = imu_error**2
        self.Q = np.array([
            [imu_var*dt**4/4, imu_var*dt**3/2, 0, 0],
            [imu_var*dt**3/2, imu_var*dt**2,   0, 0],
            [0, 0, imu_var*dt**4/4, imu_var*dt**3/2],
            [0, 0, imu_var*dt**3/2, imu_var*dt**2]
        ])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 0, 1, 0]
        ])
        self.R = np.diag(np.array(sensor_errors)**2)
        self.initial_distances = initial_distances.reshape((2,1))
    
    def predict(self, u):
        # u must be a 2x1 numpy array [[accel_imu1], [accel_imu2]]
        self.x = self.A @ self.x + self.B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q
    
    def update(self, z_raw):
        # z_raw is 2x1 numpy array of current distance measurements
        z = self.initial_distances - z_raw  # relative movement
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(len(self.x)) - K @ self.H) @ self.P
    
    def get_state(self):
        pos = [self.x[0,0], self.x[2,0]]
        vel = (self.x[1,0] + self.x[3,0]) / 2
        return pos, vel

class HorizontalKalmanFilter:
    def __init__(self, dt, imu_error, sensor_errors):
        self.dt = dt

        # State vector: [position, velocity]
        self.x = np.zeros((2, 1))

        # State transition model
        self.F = np.array([[1, dt],
                           [0, 1]])

        # Control-input model
        self.B = np.array([[0.5 * dt ** 2],
                           [dt]])

        # Measurement model (z1 = left ToF, z2 = right ToF)
        self.H = np.array([[1, 0],
                           [1, 0]])

        # Process noise covariance
        self.Q = np.array([[0.01, 0],
                           [0, 0.01]])

        # Measurement noise covariance
        self.R = np.diag(sensor_errors)

        # Initial estimate error covariance
        self.P = np.eye(2) * 100

    def predict(self, accel_x):
        u = np.array([[accel_x]])
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        y = z - np.dot(self.H, self.x)  # Measurement residual
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R  # Residual covariance
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Kalman gain
        self.x += np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = (I - np.dot(K, self.H)) @ self.P

    def get_state(self):
        return self.x[0, 0], self.x[1, 0]  # Return position and velocity

# At each timestep:
# u = vertical acceleration from IMUs (float)
# z = np.array([[sensor1_position], [sensor2_position]])

# kf.predict(u)
# kf.update(z)
# pos, vel = kf.get_state()
