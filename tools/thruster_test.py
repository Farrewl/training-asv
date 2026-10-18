"""tools/thruster_test.py — uji thruster/ESC via Pixhawk (MAVLink).

DUA mode kirim:
  servo (default) — MAV_CMD_DO_SET_SERVO per kanal (butuh SERVOx_FUNCTION
                    diset di param; paling langsung untuk bench test)
  rc              — RC_CHANNELS_OVERRIDE (lewat Mixing ArduRover; pakai
                    mode MANUAL, kapal jangan dipasang baling-baling)

PERINGATAN KEAMANAN:
  - LEPAS BALING-BALING / uji ESC tanpa beban.
  - Semua PWM kembali netral (1500) sebelum keluar; sertakan --disarm
    untuk ikut melumpuhkan motor.

Pakai:
    python tools/thruster_test.py --port COM5
    python tools/thruster_test.py --port /dev/ttyUSB0 --mode rc
    python tools/thruster_test.py --selftest          # tanpa hardware
"""

import argparse
import sys
import time

NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1100, 1900

MODE_MANUAL, MODE_HOLD, MODE_AUTO, MODE_RTL = 0, 4, 10, 11
MODE_NAMES = {MODE_MANUAL: "MANUAL", MODE_HOLD: "HOLD", MODE_AUTO: "AUTO",
              MODE_RTL: "RTL"}


def _menu(text):
    try:
        return input(text).strip()
    except (EOFError, KeyboardInterrupt):
        return "0"


class Link:
    """Pembungkus pymavlink (kering, gampang di-dry-run)."""

    def __init__(self, port=None, baud=115200, dry=False):
        self.dry = dry
        self.sysid = 1
        self.compid = 1
        self.m = None
        if dry:
            return
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(port, baud=baud,
                                            autoreconnect=True)
        hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        if hb is None:
            raise RuntimeError("tidak ada heartbeat — Pixhawk colok/port "
                               "benar? (tools/conn_check.py)")
        self.sysid = hb.get_srcSystem()
        self.compid = hb.get_srcComponent()
        self.armed = bool(hb.base_mode & 128)
        print("Pixhawk hidup: sysid=%d armed=%s" % (self.sysid, self.armed))

    def _send(self, name, fn, *args):
        if self.dry or fn is None:
            print("  [dry-run] %s%s" % (name, args))
            return
        fn(*args)

    def set_servo(self, channel, pwm):
        from pymavlink import mavutil
        if self.dry or self.m is None:
            self._send("DO_SET_SERVO", None,
                       self.sysid, self.compid,
                       mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                       channel, pwm, 0, 0, 0, 0, 0)
            return
        self._send("DO_SET_SERVO", self.m.mav.command_long_send,
                   self.sysid, self.compid,
                   mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
                   channel, pwm, 0, 0, 0, 0, 0)

    def rc_override(self, channel, pwm):
        chans = [65535] * 12          # 65535 = ignore kanal itu
        chans[channel - 1] = pwm
        if self.dry or self.m is None:
            self._send("RC_CHANNELS_OVERRIDE", None,
                       self.sysid, self.compid, *chans)
            return
        self._send("RC_CHANNELS_OVERRIDE", self.m.mav.rc_channels_override_send,
                   self.sysid, self.compid, *chans)

    def arm(self, yes=True):
        from pymavlink import mavutil
        if self.dry or self.m is None:
            self._send("ARM", None,
                       self.sysid, self.compid,
                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                       1 if yes else 0, 0, 0, 0, 0, 0, 0)
            self.armed = yes
            return
        self._send("ARM", self.m.mav.command_long_send,
                   self.sysid, self.compid,
                   mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                   1 if yes else 0, 0, 0, 0, 0, 0, 0)
        self.armed = yes

    def set_mode(self, custom_mode):
        from pymavlink import mavutil
        self._send("SET_MODE",
                   self.m.mav.command_long_send if not self.dry else None,
                   self.sysid, self.compid, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                   0, 1, custom_mode, 0, 0, 0, 0, 0)

    def neutral_all(self, channels):
        for ch in channels:
            self.out(ch, NEUTRAL)

    def out(self, channel, pwm):
        if self.mode == "servo":
            self.set_servo(channel, pwm)
        else:
            self.rc_override(channel, pwm)


def run_test(link, mode, channels):
    link.mode = mode
    print("\nMode kirim: %s | netral=%d | rentang PWM %d..%d" %
          (mode.upper(), NEUTRAL, PWM_MIN, PWM_MAX))
    print("PERINGATAN: lepas baling-baling / uji tanpa beban!\n")
    try:
        while True:
            print("--- menu ---")
            print("1 arm        2 disarm       3 set PWM kanal")
            print("4 test sequence (maju->netral->mundur->netral)")
            print("5 set mode (MANUAL/HOLD/AUTO/RTL)  6 semua netral")
            print("0 keluar")
            c = _menu("> ")
            if c == "1":
                link.arm(True)
            elif c == "2":
                link.arm(False)
            elif c == "3":
                ch = int(_menu("kanal 1-8: ") or 0)
                v = int(_menu("PWM %d-%d: " % (PWM_MIN, PWM_MAX)) or NEUTRAL)
                v = max(PWM_MIN, min(PWM_MAX, v))
                if ch in (0, *channels):
                    if ch == 0:
                        print("kanal invalid")
                        continue
                link.out(ch, v)
                print("kanal %d -> %d" % (ch, v))
            elif c == "4":
                ch = int(_menu("kanal: ") or 0)
                if ch not in channels:
                    print("kanal harus", channels)
                    continue
                for pwm, label in ((1600, "MAJU"), (NEUTRAL, "netral"),
                                   (1400, "MUNDUR"), (NEUTRAL, "netral")):
                    print("  %s -> %d (3 dtk)" % (label, pwm))
                    link.out(ch, pwm)
                    time.sleep(3 if not link.dry else 0.05)
                print("  selesai, netral")
            elif c == "5":
                m = _menu("mode %s: " % "/".join(
                    "%d=%s" % kv for kv in MODE_NAMES.items()))
                link.set_mode(int(m))
            elif c == "6":
                link.neutral_all(channels)
                print("semua netral")
            elif c == "0":
                break
    finally:
        link.neutral_all(channels)
        print("netral terkirim. sampai jumpa")


def selftest():
    print("=== SELFTEST (dry-run, tanpa hardware) ===")
    link = Link(dry=True)
    link.mode = "servo"
    link.arm(True)
    link.set_servo(1, 1600)
    link.set_servo(1, NEUTRAL)
    link.set_servo(1, 1400)
    link.neutral_all([1, 2])
    link.arm(False)
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description="Uji thruster/ESC via Pixhawk")
    ap.add_argument("--port", help="mis. COM5 / /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["servo", "rc"], default="servo")
    ap.add_argument("--channels", default="1,2",
                    help="kanal thruster dipisah koma (default: 1,2)")
    ap.add_argument("--selftest", action="store_true",
                    help="jalankan dry-run tanpa hardware")
    args = ap.parse_args()

    channels = [int(c) for c in args.channels.split(",")]
    if args.selftest:
        selftest()
        return
    if not args.port:
        ap.error("--port wajib (atau pakai --selftest)")
    try:
        link = Link(args.port, args.baud)
    except Exception as e:
        print("GAGAL: %s" % e)
        sys.exit(1)
    run_test(link, args.mode, channels)


if __name__ == "__main__":
    main()
