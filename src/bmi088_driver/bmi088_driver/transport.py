"""Register-level transports for the BMI088.

The BMI088 driver in :mod:`bmi088_driver.bmi088` only needs three primitives:
power the sensor, read N bytes from a register, and write bytes to a register.
Everything board-specific (Application Board + COINES, a Raspberry Pi, an
FT232H USB adapter, ...) lives behind the :class:`Transport` interface, so the
sensor logic and the ROS node never change when the wiring does.

The BMI088 exposes the accelerometer and the gyroscope as two *independent*
slaves, each with its own I2C address (or SPI chip-select), so every method
takes a ``dev_addr``.
"""

from __future__ import annotations

import abc
from typing import List


class Transport(abc.ABC):
    """Abstract read/write channel to a BMI088 sub-sensor."""

    @abc.abstractmethod
    def open(self) -> None:
        """Open the channel and power the sensor (VDD/VDDIO)."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release the channel and (optionally) power down the sensor."""

    @abc.abstractmethod
    def read(self, dev_addr: int, reg_addr: int, length: int) -> List[int]:
        """Read ``length`` bytes starting at ``reg_addr`` from ``dev_addr``."""

    @abc.abstractmethod
    def write(self, dev_addr: int, reg_addr: int, data: List[int]) -> None:
        """Write ``data`` bytes starting at ``reg_addr`` to ``dev_addr``."""


class CoinesTransport(Transport):
    """Path A: BMI088 Shuttle Board 3.0 on a Bosch Application Board 3.0/3.1.

    Talks to the board over USB via the Bosch ``coinespy`` package (the Python
    wrapper around the COINES SDK). Install with ``pip install coinespy`` and
    flash the ``coines_bridge`` firmware onto the Application Board first
    (see the package README).

    .. note::
       ``coinespy`` method signatures have shifted slightly across SDK
       releases. This class targets the 1.x API. If ``import coinespy`` or a
       call below fails, check ``help(coinespy.CoinesBoard)`` for the installed
       version and adjust — the BMI088 logic above this layer is unaffected.
    """

    def __init__(self, vdd: float = 3.3, vddio: float = 3.3,
                 i2c_mode: str = 'standard') -> None:
        self._vdd = vdd
        self._vddio = vddio
        self._i2c_mode = i2c_mode
        self._board = None
        self._bus = None
        self._cpy = None

    def open(self) -> None:
        import coinespy as cpy  # imported lazily so the module loads w/o hardware

        self._cpy = cpy
        board = cpy.CoinesBoard()
        board.open_comm_interface(cpy.CommInterface.USB)

        # The BMI088 latches its interface (I2C vs SPI) from its protocol-select
        # pins at power-up, so the pins must be set BEFORE VDD is applied. This
        # mirrors Bosch's COINES bmi08x example (examples/bmi08x/common/common.c).
        # BMI088 is 1.8 V / 3.3 V only — never 5 V. coinespy takes volts.
        board.set_shuttleboard_vdd_vddio_config(0.0, 0.0)  # power off first
        board.delay_milli_sec(10)

        # Select I2C: SDO low -> accel 0x18 / gyro 0x68; PS (pin 9) high -> I2C.
        out = cpy.PinDirection.OUTPUT
        pin = cpy.MultiIOPin
        board.set_pin_config(pin.SHUTTLE_PIN_SDO, out, cpy.PinValue.LOW)
        board.set_pin_config(pin.SHUTTLE_PIN_8, out, cpy.PinValue.LOW)
        board.set_pin_config(pin.SHUTTLE_PIN_9, out, cpy.PinValue.HIGH)

        self._bus = cpy.I2CBus.BUS_I2C_0
        mode = (cpy.I2CMode.FAST_MODE if self._i2c_mode == 'fast'
                else cpy.I2CMode.STANDARD_MODE)
        board.config_i2c_bus(self._bus, 0, mode)
        board.delay_milli_sec(10)

        # Now apply VDD/VDDIO — the sensor latches I2C on this power-up edge.
        board.set_shuttleboard_vdd_vddio_config(self._vdd, self._vddio)
        board.delay_milli_sec(10)
        self._board = board

    def close(self) -> None:
        if self._board is not None:
            try:
                self._board.set_shuttleboard_vdd_vddio_config(0.0, 0.0)
            finally:
                self._board.close_comm_interface()
            self._board = None

    def read(self, dev_addr: int, reg_addr: int, length: int) -> List[int]:
        data = self._board.read_i2c(self._bus, reg_addr, length, dev_addr)
        return list(data)

    def write(self, dev_addr: int, reg_addr: int, data: List[int]) -> None:
        self._board.write_i2c(self._bus, reg_addr, list(data), dev_addr)
