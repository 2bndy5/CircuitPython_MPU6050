"""
A simple test of the MPU6050 6D0F sensor
"""
import time
import board
import busio
from circuitpython_mpu6050 import MPU6050
I2C_BUS = busio.I2C(board.SCL, board.SDA)
IMU_SENSOR = MPU6050(I2C_BUS)

while True:
    try:
        ACCEL = IMU_SENSOR.acceleration
        TEMP = IMU_SENSOR.temperature
        GYRO = IMU_SENSOR.gyro
        print('temp =', TEMP)
        print('accel =', repr(ACCEL))
        print('gyro =', repr(GYRO))
        time.sleep(2)
    except KeyboardInterrupt:
        del IMU_SENSOR
        break
