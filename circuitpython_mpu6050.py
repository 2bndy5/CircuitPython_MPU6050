# The MIT License (MIT)
#
# Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
# Copyright (c) 2019 Brendan Doherty
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
=====================
circuitpython_mpu6050
=====================

The original code from `Tijndagamer's mpu6050 python module
<https://github.com/Tijndagamer/mpu6050.git>`_ was ported to use the
adafruit-circuitpython-busdevice library.

Authors: `Dr David Martin <https://github.com/davidmam>`_,
`Martijn <https://github.com/Tijndagamer>`_,
`Brendan Doherty <https://github.com/2bndy5>`_

modified by `Brendan Doherty <https://github.com/2bndy5>`_

"""
from struct import unpack_from
from adafruit_bus_device.i2c_device import I2CDevice
# pylint: disable=bad-whitespace

# Global Variables
_GRAVITY = 9.80665

# Scale Modifiers
ACCEL_SCALE_MODIFIER_2G  = 16384.0
ACCEL_SCALE_MODIFIER_4G  = 8192.0
ACCEL_SCALE_MODIFIER_8G  = 4096.0
ACCEL_SCALE_MODIFIER_16G = 2048.0

GYRO_SCALE_MODIFIER_250DEG  = 131.0
GYRO_SCALE_MODIFIER_500DEG  = 65.5
GYRO_SCALE_MODIFIER_1000DEG = 32.8
GYRO_SCALE_MODIFIER_2000DEG = 16.4

# Pre-defined ranges
ACCEL_RANGE_2G  = 0x00
ACCEL_RANGE_4G  = 0x08
ACCEL_RANGE_8G  = 0x10
ACCEL_RANGE_16G = 0x18

GYRO_RANGE_250DEG  = 0x00
GYRO_RANGE_500DEG  = 0x08
GYRO_RANGE_1000DEG = 0x10
GYRO_RANGE_2000DEG = 0x18

# MPU-6050 Registers
_PWR_MGMT_1   = 0x6B

_ACCEL_XOUT0  = 0x3B
_TEMP_OUT0    = 0x41
_GYRO_XOUT0   = 0x43

_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG  = 0x1B
# pylint: enable=bad-whitespace

def _twos_comp(val, bits):
    # Convert an unsigned integer in 2's compliment form of the specified bit
    # length to its signed integer value and return it.
    if val & (1 << (bits - 1)) != 0:
        return val - (1 << bits)
    return val

class MPU6050:
    """A driver class for the MPU6050 6 DoF (Degrees of Freedom) sensor.

    :param ~busio.I2C i2c: The I2C bus object connected to the MPU6050.

        .. note:: This object should be shared among other driver classes that use the
            same I2C bus (SDA & SCL pins) to connect to different I2C devices.

    :param int address: The MPU6050's I2C device address. In most cases this is
        the default of ``0x68``. If your scenario is different, you can specify an
        alternate address with this parameter.

    """
    def __init__(self, i2c, address=0x68):
        self._i2c = I2CDevice(i2c, address)
        # Wake up the MPU-6050 since it starts in sleep mode
        self._write_byte(_PWR_MGMT_1, 0x00)
        self._accel_scale_modifier = None
        self.accel_range = ACCEL_RANGE_2G
        self._gyro_scale_modifier = None
        self.gyro_scale = GYRO_RANGE_250DEG

    def _write_byte(self, reg, value):
        """only writes 1 byte of data"""
        if value & 0xFF: # ensure it is a single byte
            with self._i2c as i2c: # grab lock on bus
                i2c.write(bytes([reg, value]))

    def _read_bytes(self, reg, count=1):
        """ count=1 means this function will only read and return 1 byte.
        Set count to the number of bytes you want to read."""
        buf = bytearray([reg]) # first byte is the register address
        for _ in range(count):
            buf += b'\x00' # pad out buffer to length of desired bytes
        with self._i2c as i2c:
            i2c.write_then_readinto(buf, buf, out_end=1, in_start=1, in_end=count + 1)
        return buf[1:] # return only what was read

    @property
    def accel_range(self):
        """The range of the accelerometer to range."""
        return self._read_bytes(_ACCEL_CONFIG)

    @accel_range.setter
    def accel_range(self, xl_range):
        if xl_range in (ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G, ACCEL_RANGE_16G):
            # Write the new range to the ACCEL_CONFIG register
            self._write_byte(_ACCEL_CONFIG, xl_range)
            if xl_range == ACCEL_RANGE_2G:
                self._accel_scale_modifier = ACCEL_SCALE_MODIFIER_2G
            elif xl_range == ACCEL_RANGE_4G:
                self._accel_scale_modifier = ACCEL_SCALE_MODIFIER_4G
            elif xl_range == ACCEL_RANGE_8G:
                self._accel_scale_modifier = ACCEL_SCALE_MODIFIER_8G
            elif xl_range == ACCEL_RANGE_16G:
                self._accel_scale_modifier = ACCEL_SCALE_MODIFIER_16G
        else:
            raise ValueError('specified accelerometer range is undefined')

    def read_accel_raw(self):
        """Read the raw accelerometer sensor values and return it as a
            3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
            want the acceleration in nice units you probably want to use the
            accelerometer property!
        """
        # Read the accelerometer
        raw_x, raw_y, raw_z = unpack_from('>hhh', self._read_bytes(_ACCEL_XOUT0, 6))
        return (_twos_comp(raw_x, 16), _twos_comp(raw_y, 16), _twos_comp(raw_z, 16))

    @property
    def acceleration(self):
        """The accelerometer X, Y, Z axis values as a 3-tuple of
        m/s^2 values."""
        return map(lambda data: data / self._accel_scale_modifier * _GRAVITY, self.read_accel_raw())

    @property
    def gyro_scale(self):
        """The scale of the gyroscope."""
        return self._read_bytes(_GYRO_CONFIG)

    @gyro_scale.setter
    def gyro_scale(self, g_range):
        if g_range in (GYRO_RANGE_250DEG, GYRO_RANGE_500DEG,
                       GYRO_RANGE_1000DEG, GYRO_RANGE_2000DEG):
            # Write the new range to the ACCEL_CONFIG register
            self._write_byte(_GYRO_CONFIG, g_range)
            if g_range == GYRO_RANGE_250DEG:
                self._gyro_scale_modifier = GYRO_SCALE_MODIFIER_250DEG
            elif g_range == GYRO_RANGE_500DEG:
                self._gyro_scale_modifier = GYRO_SCALE_MODIFIER_500DEG
            elif g_range == GYRO_RANGE_1000DEG:
                self._gyro_scale_modifier = GYRO_SCALE_MODIFIER_1000DEG
            elif g_range == GYRO_RANGE_2000DEG:
                self._gyro_scale_modifier = GYRO_SCALE_MODIFIER_2000DEG
        else:
            raise ValueError('Specified Gyroscope scale is undefined')

    def read_gyro_raw(self):
        """Read the raw gyroscope sensor values and return it as a
            3-tuple of X, Y, Z axis values that are 16-bit unsigned values.  If you
            want the gyroscope in nice units you probably want to use the
            gyroscope property!
        """
        raw_x, raw_y, raw_z = unpack_from('>hhh', self._read_bytes(_GYRO_XOUT0, 6))
        return (_twos_comp(raw_x, 16), _twos_comp(raw_y, 16), _twos_comp(raw_z, 16))

    @property
    def gyro(self):
        """The gyroscope X, Y, Z axis values as a 3-tuple of
            degrees/second values.
        """
        return map(lambda data: data / self._gyro_scale_modifier, self.read_gyro_raw())

    def read_temp_raw(self):
        """Read the raw temperature sensor value and return it as a 16-bit
            signed value.  If you want the temperature in nice units you probably
            want to use the temperature property!
        """
        raw = self._read_bytes(_TEMP_OUT0, 2)
        return _twos_comp((raw[0] << 8) | raw[1], 16)


    @property
    def temperature(self):
        """The temperature from the onboard temperature sensor of the
            MPU-6050 in degrees Celcius."""
        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        return (self.read_temp_raw() / 340.0) + 36.53
