"""Offline checks for the BMI088 register driver (no hardware needed).

A fake transport feeds canned register bytes so we can verify chip-id checks,
the signed little-endian byte assembly, and the SI conversion factors.
"""

import math

import pytest

from bmi088_driver.bmi088 import (ACC_ADDR, ACC_CHIP_ID, ACC_CHIP_ID_VAL,
                                  ACC_DATA_START, GRAVITY, GYRO_ADDR,
                                  GYRO_CHIP_ID, GYRO_CHIP_ID_VAL,
                                  GYRO_DATA_START, BMI088, BMI088Error,
                                  _to_int16)
from bmi088_driver.transport import Transport


class FakeTransport(Transport):
    """Returns programmable bytes per (dev_addr, reg_addr); records writes."""

    def __init__(self, acc_id=ACC_CHIP_ID_VAL, gyro_id=GYRO_CHIP_ID_VAL):
        self._acc_id = acc_id
        self._gyro_id = gyro_id
        self.accel_bytes = [0, 0, 0, 0, 0, 0]
        self.gyro_bytes = [0, 0, 0, 0, 0, 0]
        self.writes = []

    def open(self):
        pass

    def close(self):
        pass

    def read(self, dev_addr, reg_addr, length):
        # Accel and gyro both expose CHIP_ID at register 0x00, so disambiguate
        # on the device address, not the register.
        if dev_addr == ACC_ADDR:
            if reg_addr == ACC_CHIP_ID:
                return [self._acc_id]
            if reg_addr == ACC_DATA_START:
                return list(self.accel_bytes)
        elif dev_addr == GYRO_ADDR:
            if reg_addr == GYRO_CHIP_ID:
                return [self._gyro_id]
            if reg_addr == GYRO_DATA_START:
                return list(self.gyro_bytes)
        return [0] * length

    def write(self, dev_addr, reg_addr, data):
        self.writes.append((dev_addr, reg_addr, list(data)))


def test_to_int16_signedness():
    assert _to_int16(0x00, 0x00) == 0
    assert _to_int16(0xFF, 0x7F) == 32767
    assert _to_int16(0x00, 0x80) == -32768
    assert _to_int16(0x00, 0xFF) == -256


def test_chip_id_mismatch_raises():
    imu = BMI088(FakeTransport(acc_id=0x00))
    with pytest.raises(BMI088Error):
        imu.open()


def test_accel_full_scale_conversion():
    # ±6 g range, full-scale positive count on Z -> ~ +6 g in m/s^2.
    t = FakeTransport()
    t.accel_bytes = [0x00, 0x00, 0x00, 0x00, 0xFF, 0x7F]  # z = 32767
    imu = BMI088(t, acc_range_g=6)
    imu.open()
    _, _, az = imu.read_accel()
    assert az == pytest.approx(6.0 * GRAVITY, rel=1e-3)


def test_gyro_full_scale_conversion():
    # ±2000 dps range, full-scale positive count on X -> ~ +2000 dps in rad/s.
    t = FakeTransport()
    t.gyro_bytes = [0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00]  # x = 32767
    imu = BMI088(t, gyro_range_dps=2000)
    imu.open()
    gx, _, _ = imu.read_gyro()
    assert gx == pytest.approx(2000.0 * math.pi / 180.0, rel=1e-3)
