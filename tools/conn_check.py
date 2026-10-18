"""tools/conn_check.py — diagnosa koneksi hardware ASV.

Cek: port serial (COM/tty) terdeteksi, heartbeat Pixhawk (MAVLink),
sinyal RC channels, gamepad (pygame opsional).

Pakai:
    python tools/conn_check.py                 # scan semua port, baud 115200+57600
    python tools/conn_check.py --port COM5     # fokus satu port
    python tools/conn_check.py --rc            # tampilkan juga RC_CHANNELS_RAW
"""

import argparse
import sys
import time

try:
    from serial.tools import list_ports
except ImportError:
    list_ports = None

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


def probe_heartbeat(port, baud, timeout):
    """Coba heartbeat satu port. Return dict hasil atau None."""
    try:
        m = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        m.close()
        if hb is None:
            return None
        return dict(port=port, baud=baud, sysid=hb.get_srcSystem(),
                    compid=hb.get_srcComponent(), type=hb.type,
                    autopilot=hb.autopilot, base_mode=hb.base_mode,
                    custom_mode=hb.custom_mode,
                    armed=bool(hb.base_mode & 128))
    except Exception as e:
        return dict(port=port, baud=baud, error=str(e))


def probe_rc(port, baud, timeout):
    """Ambil sekali RC_CHANNELS_RAW (bila vehicle kirim)."""
    try:
        m = mavutil.mavlink_connection(port, baud=baud, autoreconnect=False)
        m.recv_match(type="HEARTBEAT", blocking=True, timeout=timeout)
        msg = m.recv_match(type="RC_CHANNELS_RAW", blocking=True,
                           timeout=timeout)
        m.close()
        if msg is None:
            return None
        return dict(port=port, port_raw=msg.port, rssi=msg.rssi,
                    chans=[getattr(msg, "chan%d_raw" % i) for i in
                           range(1, 9)])
    except Exception:
        return None


def check_gamepad():
    """pygame opsional — tidak fatal bila tak ada."""
    try:
        import pygame
    except ImportError:
        return "pygame tidak terpasang (opsional): pip install pygame"
    pygame.init()
    try:
        n = pygame.joystick.get_count()
        if n == 0:
            return "gamepad: tidak terdeteksi"
        names = [pygame.joystick.Joystick(i).get_name()
                 for i in range(n)]
        return "gamepad: %d terdeteksi — %s" % (n, "; ".join(names))
    finally:
        pygame.quit()


def main():
    ap = argparse.ArgumentParser(description="Diagnosa koneksi ASV")
    ap.add_argument("--port", help="fokus satu port (mis. COM5 / /dev/ttyUSB0)")
    ap.add_argument("--bauds", default="115200,57600",
                    help="daftar baud dipisah koma (default: 115200,57600)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="timeout heartbeat per port (detik)")
    ap.add_argument("--rc", action="store_true",
                    help="tampilkan juga contoh RC_CHANNELS_RAW")
    args = ap.parse_args()

    print("=== 1. Port serial ===")
    ports = []
    if list_ports is None:
        print("pyserial tidak terpasang — pip install pyserial")
    else:
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            print("tidak ada port terdeteksi (Pixhawk/lidar belum colok?)")
        for p in list_ports.comports():
            print("  %-20s %s  %s" % (p.device, p.description, p.hwid))

    if mavutil is None:
        print("\npymavlink tidak terpasang — pip install pymavlink")
        sys.exit(1)

    if args.port:
        targets = [args.port]
    else:
        targets = ports
    if not targets:
        print("\n=== 2. MAVLink heartbeat ===\n  dilewati (tak ada port)")
    else:
        print("\n=== 2. MAVLink heartbeat ===")
        found = False
        for port in targets:
            for baud in (int(b) for b in args.bauds.split(",")):
                r = probe_heartbeat(port, baud, args.timeout)
                if r and "error" not in r:
                    found = True
                    print("  %s @ %d baud -> HIDUP sysid=%d compid=%d "
                          "type=%d autopilot=%d armed=%s custom_mode=%d" %
                          (port, baud, r["sysid"], r["compid"], r["type"],
                           r["autopilot"], r["armed"], r["custom_mode"]))
                    if args.rc:
                        rc = probe_rc(port, baud, args.timeout)
                        if rc:
                            print("    RC port=%d rssi=%d chans=%s" %
                                  (rc["port_raw"], rc["rssi"], rc["chans"]))
                        else:
                            print("    RC_CHANNELS_RAW tidak diterima")
                    break
            else:
                err = r.get("error") if r else "tidak ada heartbeat"
                print("  %s -> MATI (%s)" % (port, err or "timeout"))
        if not found:
            print("  tips: cek kabel USB, driver CP2102/FTDI, port di "
                  "Device Manager / dmesg")

    print("\n=== 3. Gamepad ===")
    print("  " + check_gamepad())


if __name__ == "__main__":
    main()
