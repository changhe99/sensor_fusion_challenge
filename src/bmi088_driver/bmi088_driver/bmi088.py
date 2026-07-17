"""Transport-agnostic BMI088 register driver.

The BMI088 is two independent MEMS sensors in one package — an accelerometer
and a gyroscope, each on its own I2C address. This module configures both and
returns readings already converted to SI units (m/s^2 and rad/s), so the ROS
node only has to stamp and publish them.

Register names and conversion formulas follow the Bosch BMI088 datasheet
(rev 1.7). Byte-level I/O is delegated to a :class:`~bmi088_driver.transport.Transport`.
"""

from __future__ import annotations

import math
import time
from typing import Tuple

from bmi088_driver.transport import Transport

# --- Accelerometer (default I2C addr 0x18; SDO high -> 0x19) ----------------
ACC_ADDR = 0x18
ACC_CHIP_ID = 0x00          # expect 0x1E
ACC_DATA_START = 0x12       # X_LSB; 6 bytes X/Y/Z little-endian
ACC_CONF = 0x40
ACC_RANGE = 0x41
ACC_PWR_CONF = 0x7C         # 0x00 active, 0x03 suspend
ACC_PWR_CTRL = 0x7D         # 0x04 accel on, 0x00 off
ACC_SOFTRESET = 0x7E        # 0xB6
ACC_CHIP_ID_VAL = 0x1E

# ACC_RANGE register value -> full-scale in g
ACC_RANGE_TO_G = {0x00: 3.0, 0x01: 6.0, 0x02: 12.0, 0x03: 24.0}
G_TO_RANGE = {3: 0x00, 6: 0x01, 12: 0x02, 24: 0x03}
# ACC_CONF low nibble (ODR) for common rates
ACC_ODR = {12: 0x05, 25: 0x06, 50: 0x07, 100: 0x08,
           200: 0x09, 400: 0x0A, 800: 0x0B, 1600: 0x0C}
ACC_BWP_NORMAL = 0x0A       # ACC_CONF high nibble: normal filter

# --- Gyroscope (default I2C addr 0x68; SDO high -> 0x69) --------------------
GYRO_ADDR = 0x68
GYRO_CHIP_ID = 0x00         # expect 0x0F
GYRO_DATA_START = 0x02      # RATE_X_LSB; 6 bytes
GYRO_RANGE = 0x0F
GYRO_BANDWIDTH = 0x10
GYRO_LPM1 = 0x11            # 0x00 normal
GYRO_SOFTRESET = 0x14       # 0xB6
GYRO_CHIP_ID_VAL = 0x0F

# GYRO_RANGE register value -> full-scale in deg/s
GYRO_RANGE_TO_DPS = {0x00: 2000.0, 0x01: 1000.0, 0x02: 500.0,
                     0x03: 250.0, 0x04: 125.0}
DPS_TO_RANGE = {2000: 0x00, 1000: 0x01, 500: 0x02, 250: 0x03, 125: 0x04}
# GYRO_BANDWIDTH register value -> output data rate in Hz
GYRO_ODR = {2000: 0x00, 1000: 0x02, 400: 0x03, 200: 0x04, 100: 0x05}

GRAVITY = 9.80665           # m/s^2
DEG2RAD = math.pi / 180.0

Vector3 = Tuple[float, float, float]


def _to_int16(lsb: int, msb: int) -> int:
    """Combine two bytes into a signed 16-bit little-endian integer."""
    val = (msb << 8) | lsb
    return val - 65536 if val >= 32768 else val


class BMI088Error(RuntimeError):
    """Raised on chip-ID mismatch or a failed transaction."""


class BMI088:
    def __init__(self, transport: Transport,
                 acc_range_g: int = 6, acc_odr_hz: int = 200,
                 gyro_range_dps: int = 2000, gyro_odr_hz: int = 200,
                 acc_addr: int = ACC_ADDR, gyro_addr: int = GYRO_ADDR) -> None:
        if acc_range_g not in G_TO_RANGE:
            raise ValueError(f'acc_range_g must be one of {sorted(G_TO_RANGE)}')
        if acc_odr_hz not in ACC_ODR:
            raise ValueError(f'acc_odr_hz must be one of {sorted(ACC_ODR)}')
        if gyro_range_dps not in DPS_TO_RANGE:
            raise ValueError(f'gyro_range_dps must be one of {sorted(DPS_TO_RANGE)}')
        if gyro_odr_hz not in GYRO_ODR:
            raise ValueError(f'gyro_odr_hz must be one of {sorted(GYRO_ODR)}')

        self._t = transport
        self._acc_addr = acc_addr
        self._gyro_addr = gyro_addr
        self._acc_range_reg = G_TO_RANGE[acc_range_g]
        self._acc_odr_reg = ACC_ODR[acc_odr_hz]
        self._gyro_range_reg = DPS_TO_RANGE[gyro_range_dps]
        self._gyro_odr_reg = GYRO_ODR[gyro_odr_hz]
        # Cached full-scale factors for fast conversion in the hot path.
        self._acc_g_per_count = ACC_RANGE_TO_G[self._acc_range_reg] / 32768.0
        self._gyro_dps_per_count = GYRO_RANGE_TO_DPS[self._gyro_range_reg] / 32768.0

    # -- lifecycle -----------------------------------------------------------
    def open(self) -> None:
        self._t.open()
        # Give VDD/VDDIO time to settle before talking to either sub-sensor.
        time.sleep(0.05)
        self._init_accel()
        self._init_gyro()

    def close(self) -> None:
        self._t.close()

    # -- configuration -------------------------------------------------------
    def _verify_chip_id(self, addr: int, expected: int, name: str,
                        retries: int = 5) -> None:
        """Read CHIP_ID (reg 0x00) until it matches, else raise.

        The first read right after a soft reset (or after an unclean prior
        session) can return 0x00, so retry a few times before giving up.
        """
        chip_id = -1
        for _ in range(retries):
            chip_id = self._t.read(addr, 0x00, 1)[0]
            if chip_id == expected:
                return
            time.sleep(0.01)
        raise BMI088Error(
            f'{name} chip id 0x{chip_id:02X} != 0x{expected:02X}')

    def _init_accel(self) -> None:
        # Soft reset; the retried CHIP_ID read below also leaves the accel in
        # I2C mode (the first post-reset read is the dummy).
        self._t.write(self._acc_addr, ACC_SOFTRESET, [0xB6])
        time.sleep(0.05)
        self._verify_chip_id(self._acc_addr, ACC_CHIP_ID_VAL, 'accel')

        # Power on: enable the accel, then switch to active mode.
        self._t.write(self._acc_addr, ACC_PWR_CTRL, [0x04])
        time.sleep(0.05)
        self._t.write(self._acc_addr, ACC_PWR_CONF, [0x00])
        time.sleep(0.005)

        self._t.write(self._acc_addr, ACC_CONF,
                      [(ACC_BWP_NORMAL << 4) | self._acc_odr_reg])
        self._t.write(self._acc_addr, ACC_RANGE, [self._acc_range_reg])
        time.sleep(0.005)

    def _init_gyro(self) -> None:
        self._t.write(self._gyro_addr, GYRO_SOFTRESET, [0xB6])
        time.sleep(0.05)
        # The first read right after a gyro soft reset returns 0x00; the retry
        # loop rides past it.
        self._verify_chip_id(self._gyro_addr, GYRO_CHIP_ID_VAL, 'gyro')

        self._t.write(self._gyro_addr, GYRO_LPM1, [0x00])       # normal mode
        time.sleep(0.005)
        self._t.write(self._gyro_addr, GYRO_RANGE, [self._gyro_range_reg])
        self._t.write(self._gyro_addr, GYRO_BANDWIDTH, [self._gyro_odr_reg])
        time.sleep(0.005)

    # -- reads ---------------------------------------------------------------
    def read_accel(self) -> Vector3:
        """Return (ax, ay, az) in m/s^2."""
        d = self._t.read(self._acc_addr, ACC_DATA_START, 6)
        ax = _to_int16(d[0], d[1]) * self._acc_g_per_count * GRAVITY
        ay = _to_int16(d[2], d[3]) * self._acc_g_per_count * GRAVITY
        az = _to_int16(d[4], d[5]) * self._acc_g_per_count * GRAVITY
        return ax, ay, az

    def read_gyro(self) -> Vector3:
        """Return (gx, gy, gz) in rad/s."""
        d = self._t.read(self._gyro_addr, GYRO_DATA_START, 6)
        gx = _to_int16(d[0], d[1]) * self._gyro_dps_per_count * DEG2RAD
        gy = _to_int16(d[2], d[3]) * self._gyro_dps_per_count * DEG2RAD
        gz = _to_int16(d[4], d[5]) * self._gyro_dps_per_count * DEG2RAD
        return gx, gy, gz
