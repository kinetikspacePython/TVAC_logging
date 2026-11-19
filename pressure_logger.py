"""
Pressure logger - Pfeiffer Vacuum RS-485 protocol (OmniControl / DigiLine / TC 400 bus)

Polls parameter 740 (Pressure) from one device on the RS-485 bus and logs it to CSV.
Run from VS Code (edit USER SETTINGS, then Run) or from the command line.
Stop with Ctrl+C - every line is flushed to disk immediately, so nothing is lost.

Requires:  pip install pyserial
"""
import csv
import datetime as dt
import os
import sys
import time

import serial  # pyserial

# ============================================================================
# USER SETTINGS
# ============================================================================
PORT = "COM3"            # USB/RS-485 converter port (Device Manager -> Ports)
BAUD = 9600              # Pfeiffer default: 9600, 8N1
ADDRESS = 1              # RS-485 address of the gauge/device (check in OmniControl menu)
PARAM = 740              # 740 = Pressure [hPa] (Pfeiffer parameter list)
INTERVAL_S = 1.0         # logging interval [s]
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
# ============================================================================


def checksum(s: str) -> str:
    """Pfeiffer checksum: sum of ASCII codes of all preceding chars, mod 256, 3 digits."""
    return f"{sum(s.encode('ascii')) % 256:03d}"


def build_query(address: int, param: int) -> bytes:
    body = f"{address:03d}00{param:03d}02=?"
    return (body + checksum(body) + "\r").encode("ascii")


def parse_response(raw: bytes, address: int, param: int) -> str:
    """Return the data field of a valid response, else raise ValueError."""
    s = raw.decode("ascii", errors="replace").strip("\r\n")
    if len(s) < 13:
        raise ValueError(f"short reply: {s!r}")
    body, cs = s[:-3], s[-3:]
    if checksum(body) != cs:
        raise ValueError(f"checksum mismatch: {s!r}")
    if int(body[0:3]) != address or int(body[5:8]) != param:
        raise ValueError(f"reply from other address/parameter: {s!r}")
    n = int(body[8:10])
    data = body[10:10 + n]
    if data in ("NO_DEF", "_RANGE", "_LOGIC"):
        raise ValueError(f"device error: {data}")
    return data


def decode_u_expo_new(data: str) -> float:
    """u_expo_new: 4-digit mantissa (x.xxx) + 2-digit exponent offset by 20.
    e.g. '100023' -> 1.000e3 ; '150016' -> 1.500e-4"""
    mant = int(data[0:4]) / 1000.0
    expo = int(data[4:6]) - 20
    return mant * 10.0 ** expo


def read_pressure(ser: serial.Serial) -> tuple[float, str]:
    ser.reset_input_buffer()
    ser.write(build_query(ADDRESS, PARAM))
    raw = ser.read_until(b"\r")
    if not raw:
        raise TimeoutError("no reply")
    data = parse_response(raw, ADDRESS, PARAM)
    return decode_u_expo_new(data), data


def main():
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(OUT_DIR, f"pressure_log_{stamp}.csv")
    print(f"Port {PORT} @ {BAUD}, address {ADDRESS:03d}, parameter {PARAM}")
    print(f"Logging every {INTERVAL_S} s to {out_path}  (Ctrl+C to stop)\n")

    with serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1, timeout=1.0) as ser, \
            open(out_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp", "elapsed_s", "pressure_hPa", "pressure_Pa", "raw", "status"])
        t0 = time.monotonic()
        next_t = t0
        try:
            while True:
                now = dt.datetime.now().isoformat(timespec="milliseconds")
                el = time.monotonic() - t0
                try:
                    p, raw = read_pressure(ser)
                    w.writerow([now, f"{el:.2f}", f"{p:.4e}", f"{p * 100:.4e}", raw, "OK"])
                    print(f"{now}  {p:.3e} hPa")
                except (ValueError, TimeoutError, serial.SerialException) as e:
                    w.writerow([now, f"{el:.2f}", "", "", "", str(e)])
                    print(f"{now}  ERROR: {e}", file=sys.stderr)
                fh.flush()
                next_t += INTERVAL_S
                time.sleep(max(0.0, next_t - time.monotonic()))
        except KeyboardInterrupt:
            print(f"\nStopped. Data saved to {out_path}")


if __name__ == "__main__":
    main()
