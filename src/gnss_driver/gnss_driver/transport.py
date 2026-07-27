"""Byte-stream transports for the u-blox ZED-F9P (ArduSimple RTK).

The GNSS driver in :mod:`gnss_driver.zed_f9p` only needs a line-oriented byte
stream: open the link, tell whether bytes are waiting, read one newline-
terminated sentence, and close. Everything board-specific (which serial port,
USB CDC-ACM vs a UART, a TCP/NTRIP bridge, ...) lives behind the
:class:`Transport` interface, so the NMEA parsing and the ROS node never change
when the wiring does.

Unlike the BMI088 (a register device we *poll*), the ZED-F9P *streams* — it
emits NMEA sentences on its own schedule (1-10 Hz). So the primitive here is
``readline`` over a buffered stream, not a register read.
"""

from __future__ import annotations

import abc


class Transport(abc.ABC):
    """Abstract line-oriented byte channel to the receiver."""

    @abc.abstractmethod
    def open(self) -> None:
        """Open the channel."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release the channel."""

    @abc.abstractmethod
    def has_data(self) -> bool:
        """True if at least one byte is already buffered (a non-blocking peek).

        Lets the reader drain everything queued without blocking on an empty
        buffer for the read timeout on every poll.
        """

    @abc.abstractmethod
    def readline(self) -> bytes:
        """Read one newline-terminated line; ``b''`` on timeout/EOF."""


class SerialTransport(Transport):
    """Path A (default): ZED-F9P over USB, enumerating as a CDC-ACM serial port.

    The ArduSimple simpleRTK2B presents the ZED-F9P's USB as ``/dev/ttyACMx``.
    On this machine the Bosch IMU board is ``ttyACM0`` and the GNSS is
    ``ttyACM1`` — but that ordering depends on enumeration, so prefer the stable
    by-id path (see ``config/gnss.yaml``):

        /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00

    On a USB CDC-ACM link the baud rate is a formality (ignored by the virtual
    UART); it only matters if you wire the ZED-F9P's physical UART instead.
    Install pyserial with ``pip install pyserial`` (already in the container).
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout: float = 0.1) -> None:
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._ser = None

    def open(self) -> None:
        import serial  # imported lazily so the module loads w/o pyserial/hardware

        self._ser = serial.Serial(
            self._port, self._baudrate, timeout=self._timeout)
        # Drop anything that arrived before we started listening so the first
        # sentence we parse is fresh, not a stale partial line.
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def has_data(self) -> bool:
        return self._ser is not None and self._ser.in_waiting > 0

    def readline(self) -> bytes:
        return self._ser.readline()
