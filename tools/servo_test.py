"""tools/servo_test.py — sweep & manual set servo per kanal (bench test).

Pakai MAV_CMD_DO_SET_SERVO — tidak pakai rc override. Sangat berguna untuk
rudder, winch, atau aktuator lain sebelum uji thruster.

Pakai:
    python tools/servo_test.py --port COM5 --channel 1
    python tools/servo_test.py --port /dev/ttyUSB0 --selftest
"""

import argparse
import sys
import time

NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1000, 2000


class Link:
    def __init__(self, port=None, baud=115200, dry=False):
        self.dry = dry
        self.sysid = 1
        self.compid = 1
        if dry:
            return
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection(port, baud=baud)
        hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=10)
        if hb is None:
            raise RuntimeError("tidak ada heartbeat (cek --port)")
        self.sysid = hb.get_srcSystem()

    def set_servo(self, ch, pwm):
        from pymavlink import mavutil
        if self.dry or self.m is None:
            print("  [dry-run] DO_SET_SERVO ch=%d pwm=%d" % (ch, pwm))
            return
        self.m.mav.command_long_send(self.sysid, 1,
                                     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                     0, ch, pwm, 0, 0, 0, 0, 0)


def sweep(link, ch, lo=1100, hi=1900, step=50, hold=0.8):
    for pwm in range(lo, hi + 1, step):
        print("ch%d -> %d" % (ch, pwm))
        link.set_servo(ch, pwm)
        time.sleep(hold)
    for pwm in range(hi, lo - 1, -step):
        print("ch%d -> %d" % (ch, pwm))
        link.set_servo(ch, pwm)
        time.sleep(hold)
    link.set_servo(ch, NEUTRAL)
    print("ch%d -> NEUTRAL (%d)" % (ch, NEUTRAL))


def manual(link, ch):
    try:
        while True:
            s = input("PWM (1000-2000) / s=sweep / q=quit: ").strip().lower()
            if s == "q":
                break
            if s == "s":
                sweep(link, ch)
                continue
            try:
                pwm = int(s)
            except ValueError:
                continue
            pwm = max(PWM_MIN, min(PWM_MAX, pwm))
            link.set_servo(ch, pwm)
    finally:
        link.set_servo(ch, NEUTRAL)


def selftest():
    l = Link(dry=True)
    sweep(l, 1, lo=1500, hi=1550, step=25, hold=0.01)
    print("selftest OK")


def main():
    ap = argparse.ArgumentParser(description="Servo sweep/test via DO_SET_SERVO")
    ap.add_argument("--port", help="COMx atau /dev/ttyUSBx")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--channel", type=int, default=1)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        selftest()
        return
    if not args.port:
        ap.error("--port wajib (atau --selftest)")
    l = Link(args.port, args.baud)
    manual(l, args.channel)


if __name__ == "__main__":
    main()
