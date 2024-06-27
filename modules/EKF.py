import smbus2
import time
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF
from filterpy.common import Q_discrete_white_noise
import matplotlib.pyplot as plt
import matplotlib

matplotlib.use('Agg')  # Use the Agg backend

def read_data():
    # Create an SMBus instance
    bus = smbus2.SMBus(1)  # 1 indicates /dev/i2c-1

    # Define the I2C address of the device
    DEVICE_ADDRESS = 0x68  # Replace with your device's I2C address

    CONTROL_REGISTER_ADDRESS = 0X6B

    ACC_X_MSB = 0x3B
    ACC_X_LSB = 0x3C
    ACC_Y_MSB = 0x3D
    ACC_Y_LSB = 0x3E
    ACC_Z_MSB = 0x3F
    ACC_Z_LSB = 0x40

    GYRO_X_MSB = 0x43
    GYRO_X_LSB = 0x44
    GYRO_Y_MSB = 0x45
    GYRO_Y_LSB = 0x46
    GYRO_Z_MSB = 0x47
    GYRO_Z_LSB = 0x48

    try:
        bus.write_byte_data(DEVICE_ADDRESS, CONTROL_REGISTER_ADDRESS, 0x00)
        # Read the acceleration data
        acc_x_msb = bus.read_byte_data(DEVICE_ADDRESS, ACC_X_MSB)
        acc_x_lsb = bus.read_byte_data(DEVICE_ADDRESS, ACC_X_LSB)
        acc_y_msb = bus.read_byte_data(DEVICE_ADDRESS, ACC_Y_MSB)
        acc_y_lsb = bus.read_byte_data(DEVICE_ADDRESS, ACC_Y_LSB)
        acc_z_msb = bus.read_byte_data(DEVICE_ADDRESS, ACC_Z_MSB)
        acc_z_lsb = bus.read_byte_data(DEVICE_ADDRESS, ACC_Z_LSB)

        # Read the gyroscope data
        gyro_x_msb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_X_MSB)
        gyro_x_lsb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_X_LSB)
        gyro_y_msb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_Y_MSB)
        gyro_y_lsb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_Y_LSB)
        gyro_z_msb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_Z_MSB)
        gyro_z_lsb = bus.read_byte_data(DEVICE_ADDRESS, GYRO_Z_LSB)

        # 16-bit two's complement signed representation
        acc_x = (acc_x_msb << 8) | acc_x_lsb
        acc_y = (acc_y_msb << 8) | acc_y_lsb
        acc_z = (acc_z_msb << 8) | acc_z_lsb

        if acc_x & 0x8000:
            acc_x -= 1 << 16
        
        if acc_y & 0x8000:
            acc_y -= 1 << 16
        
        if acc_z & 0x8000:
            acc_z -= 1 << 16

        # 16-bit two's complement signed representation
        gyro_x = (gyro_x_msb << 8) | gyro_x_lsb
        gyro_y = (gyro_y_msb << 8) | gyro_y_lsb
        gyro_z = (gyro_z_msb << 8) | gyro_z_lsb

        if gyro_x & 0x8000:
            gyro_x -= 1 << 16
        
        if gyro_y & 0x8000:
            gyro_y -= 1 << 16

        if gyro_z & 0x8000:
            gyro_z -= 1 << 16
            
        print(f"Acceleration: X={acc_x}, Y={acc_y}, Z={acc_z}")
        print(f"Gyroscope: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
        print()

        bus.close()
        time.sleep(0.5)
        return [gyro_x, gyro_y, gyro_z], [acc_x, acc_y, acc_z]
                
    except KeyboardInterrupt:
        print("Program interrupted by user")
        bus.close()
        exit(1)

def fx(x, dt):
    """ State transition function for constant velocity model.
    x is the state vector [x, y, z, vx, vy, vz]
    dt is the time step
    """
    F = np.array([[1, 0, 0, dt, 0,  0],
                  [0, 1, 0, 0,  dt, 0],
                  [0, 0, 1, 0,  0,  dt],
                  [0, 0, 0, 1,  0,  0],
                  [0, 0, 0, 0,  1,  0],
                  [0, 0, 0, 0,  0,  1]])
    return np.dot(F, x)
 
def hx(x):
    """ Measurement function - we only measure position (x, y, z) """
    return x[:3]
 
def jacobian_F(x, dt):
    """ Jacobian of the state transition function """
    return np.array([[1, 0, 0, dt, 0,  0],
                     [0, 1, 0, 0,  dt, 0],
                     [0, 0, 1, 0,  0,  dt],
                     [0, 0, 0, 1,  0,  0],
                     [0, 0, 0, 0,  1,  0],
                     [0, 0, 0, 0,  0,  1]])
 
def jacobian_H(x):
    """ Jacobian of the measurement function """
    return np.array([[1, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0]])
 
# Define the filter
dt = 1.0  # time step

ekf = EKF(dim_x=6, dim_z=3)

# Initial state [x, y, z, vx, vy, vz]
initial_stategyro, initial_stateacc = read_data()
ekf.x = np.array([initial_stategyro[0], initial_stategyro[1], initial_stategyro[2], 0., 0., 0.]).T
 
# Covariance matrix
ekf.P *= 10.
 
# Measurement noise
ekf.R = np.eye(3) * 5
 
# Process noise
ekf.Q = Q_discrete_white_noise(dim=3, dt=dt, var=0.1, block_size=2)

# Lists to store values for plotting
predicted_states = []
updated_states = []
covariances = []
measurements = []

for _ in range(100):
    # Generate simulated measurements
    true_states, measurement = read_data()
    measurements.append(measurement)
    
    ekf.F = jacobian_F(ekf.x, dt)

    # Prediction step
    ekf.predict()
    predicted_states.append(ekf.x[:3].copy())
 
    # Update step
    ekf.update(np.array(measurement), HJacobian=jacobian_H, Hx=hx)
    updated_states.append(ekf.x[:3].copy())
    covariances.append(ekf.P.copy())
 
    print(f'Post update state: {ekf.x}')
    print(f'Post update covariance: {ekf.P}')
 
# Convert lists to numpy arrays for plotting
measurements = np.array(measurements)
predicted_states = np.array(predicted_states)
updated_states = np.array(updated_states)

# Plotting
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot measured positions
ax.scatter(measurements[:, 0], measurements[:, 1], measurements[:, 2], c='r', marker='o', label='Measurements')

# Plot predicted positions
ax.plot(predicted_states[:, 0], predicted_states[:, 1], predicted_states[:, 2], 'b--', label='Predicted')

# Plot updated positions
ax.plot(updated_states[:, 0], updated_states[:, 1], updated_states[:, 2], 'g-', label='Updated')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
plt.show()

plt.savefig('ekf.png')

# Plotting covariance matrices as heatmaps
fig, axs = plt.subplots(1, len(covariances), figsize=(15, 5))
for i, P in enumerate(covariances):
    im = axs[i].imshow(P, cmap='hot', interpolation='nearest')
    axs[i].set_title(f'Covariance {i+1}')
    fig.colorbar(im, ax=axs[i])

plt.show()

plt.savefig('ekf_covariance.png')

