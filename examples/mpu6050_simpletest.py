"""
A simple test of the MPU6050 6D0F sensor
"""
# pylint: disable=invalid-name
import time
import board
from circuitpython_mpu6050 import MPU6050
I2C_BUS = board.I2C()
sensor = MPU6050(I2C_BUS)

while True:
    try:
        accel_x, accel_y, accel_z = sensor.acceleration
        temp = sensor.temperature
        gyro_x, gyro_y, gyro_z = sensor.gyro
        print('temp =', temp)
        print('accel =', accel_x, accel_y, accel_z)
        print('gyro =', gyro_x, gyro_y, gyro_z)
        time.sleep(2)
    except KeyboardInterrupt:
        del sensor
        break
