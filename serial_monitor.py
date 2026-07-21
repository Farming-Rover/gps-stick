"""Print everything read from the GNSS serial port (infinite loop).

Usage:
  python serial_monitor.py
  python serial_monitor.py /dev/ttyUSB0
"""

import sys
import time

import serial

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 115200


def iter_nmea_lines(raw_line):
    """Split glued NMEA like '$GNGGA,...0$GNGSA,...' into separate lines."""
    line = raw_line.strip()
    if not line:
        return
    if line.count("$") <= 1:
        yield line
        return
    for part in line.lstrip("$").split("$"):
        if part:
            yield "$" + part


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    print(f"Opening {port} @ {BAUDRATE} baud (Ctrl+C to stop)...")

    ser = serial.Serial(port, BAUDRATE, timeout=1)
    try:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("ascii", errors="replace").strip()
            if not text:
                continue
            for sentence in iter_nmea_lines(text):
                print(sentence)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
