"""lidar/rplidar.py — driver minimal LiDAR 2D protokol Slamtec RPLIDAR (A1/A2/A3).

Hanya butuh pyserial. Protokol (ringkas):
- Perintah: byte 0xA5 lalu opcode (STOP=0x21, RESET=0x20, SCAN=0x25,
  GET_HEALTH=0x09, GET_INFO=0x06).
- Respons diawali deskriptor 5 byte:
    b0=0x5A, b1=0x5A (mode response), b2=len_lo, b3=len_hi,
    b4=checksum (XOR b0..b3).
  Khusus MODE_SCAN: len=5 & TIDAK ada payload — len artinya ukuran
  tiap paket berikutnya (stream contoh 5 byte tanpa henti).
- Tiap contoh scan 5 byte:
    b0: kualitas (>>2), start-bit revolusi baru (bit1)
    b1|b2<<8: angle_q6  -> derajat = q6 / 64
    b3|b4<<8: distance_q2 -> mm = q2 / 4 (0 = invalid)

Merek lidar tak dikenal? Pakai tools/lidar_test.py --raw untuk dump
hex, lalu cocokkan pola dengan protokol ini.

Selftest (tanpa hardware):  python -m lidar.rplidar
"""

CMD_PREFIX = 0xA5
CMD_STOP = bytes([0xA5, 0x21])
CMD_RESET = bytes([0xA5, 0x20])
CMD_SCAN = bytes([0xA5, 0x25])
CMD_HEALTH = bytes([0xA5, 0x09])
CMD_INFO = bytes([0xA5, 0x06])

DESC_SYNC = 0x5A
DESC_MODE_RESPONSE = 0x5A
DESC_TYPE_SCAN = 4

_HEALTH_NAMES = {0: "OK", 1: "WARNING", 2: "ERROR"}


class LidarError(RuntimeError):
    """Koneksi/protokol lidar bermasalah."""


def parse_response_descriptor(buf):
    """5 byte deskriptor -> panjang payload (byte)."""
    if len(buf) != 5 or buf[0] != DESC_SYNC:
        raise LidarError("deskriptor tak valid: %s" % buf.hex())
    if (buf[0] ^ buf[1] ^ buf[2] ^ buf[3]) != buf[4]:
        raise LidarError("checksum deskriptor salah: %s" % buf.hex())
    return (buf[3] & 0x03) << 8 | buf[2]


def parse_scan_sample(buf5):
    """5 byte contoh scan -> (start_rev, angle_deg, dist_mm, quality)."""
    b0, alo, ahi, dlo, dhi = buf5
    start = bool(b0 & 0x02)
    quality = b0 >> 2
    angle = ((ahi << 8) | alo) / 64.0 % 360.0
    dist = ((dhi << 8) | dlo) / 4.0
    return start, angle, dist, quality


def parse_scan_stream(buf):
    """Parse arus byte ber-aliran 5 byte. Return (samples, sisa_buf).

    samples: list of (start_rev, angle_deg, dist_mm, quality)
    """
    nfull = len(buf) - len(buf) % 5
    out = [parse_scan_sample(buf[i:i + 5]) for i in range(0, nfull, 5)]
    return out, buf[nfull:]


class FakeSerial:
    """Serial palsu untuk selftest: mengeluarkan byte dari buffer."""

    def __init__(self, data=b""):
        self._data = data
        self._pos = 0
        self.dtr = False
        self.rts = False

    def read(self, n):
        chunk = self._data[self._pos:self._pos + n]
        self._pos += len(chunk)
        return chunk

    def write(self, _b):
        return len(_b)

    def reset_input_buffer(self):
        pass

    def close(self):
        pass


class RPLidar:
    """Driver RPLIDAR lewat serial. Contoh:

        lidar = RPLidar("/dev/ttyUSB0")   # Windows: "COM5"
        lidar.open()
        print(lidar.health(), lidar.info())
        for scan in lidar.iter_scans():   # list[(angle_deg, dist_mm, quality)]
            ...
        lidar.close()
    """

    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser = None

    # --- koneksi ---
    def open(self):
        import serial  # lazy: selftest/jalan tanpa pyserial pun bisa
        self._ser = serial.Serial(
            self.port, self.baudrate, timeout=self.timeout)
        return self

    def attach(self, serial_obj):
        """Pakai objek serial yang sudah terbuka (untuk uji/komposisi)."""
        self._ser = serial_obj
        return self

    def close(self):
        if self._ser is not None:
            try:
                self.set_motor(False)
            except Exception:
                pass
            self._ser.close()
            self._ser = None

    def _need(self):
        if self._ser is None:
            raise LidarError("serial belum terbuka (panggil open()/attach())")
        return self._ser

    # --- perintah dasar ---
    def _exchange(self, cmd, expect_payload=True):
        ser = self._need()
        ser.reset_input_buffer()
        ser.write(cmd)
        desc = ser.read(5)
        if len(desc) != 5:
            raise LidarError("tidak ada respons (cek port/baud/merek lidar)")
        dlen = parse_response_descriptor(desc)
        if not expect_payload:
            return b""
        payload = ser.read(dlen)
        if len(payload) != dlen:
            raise LidarError("payload terpotong (%d/%d byte)" %
                             (len(payload), dlen))
        return payload

    def health(self):
        """-> (status_str, error_code)."""
        p = self._exchange(CMD_HEALTH)
        return _HEALTH_NAMES.get(p[0], p[0]), p[1] | p[2] << 8

    def info(self):
        """-> dict(firmware, hardware, serial)."""
        p = self._exchange(CMD_INFO)
        fw = "%d.%d" % (p[1], p[0])
        return dict(firmware=fw, hardware=p[2], serial=p[10:18].hex())

    def stop(self):
        if self._ser is not None:
            try:
                self._exchange(CMD_STOP, expect_payload=False)
            except LidarError:
                pass  # stop tetap dikirim; tak semua firmware balas

    def reset(self):
        self._exchange(CMD_RESET, expect_payload=False)

    def set_motor(self, on):
        """Motor A1/A2 umumnya di-power via DTR/RTS adapter USB-serial."""
        ser = self._need()
        try:
            ser.dtr = bool(on)
        except Exception:
            pass
        try:
            ser.rts = not on  # sebagian adapter: RTS aktif-rendah
        except Exception:
            pass

    # --- scan ---
    def start_scan(self):
        """Mulai mode scan. Deskriptor scan (len=5) tanpa payload."""
        ser = self._need()
        ser.reset_input_buffer()
        ser.write(CMD_SCAN)
        desc = ser.read(5)
        if len(desc) != 5:
            raise LidarError("SCAN tak dijawab (cek motor menyala/health)")
        dlen = parse_response_descriptor(desc)
        if dlen != 5:
            raise LidarError("bukan deskriptor scan (len=%d)" % dlen)

    def iter_scans(self):
        """Generator: yield list[(angle_deg, dist_mm, quality)] per revolusi.

        Titik distance0 (invalid) dibuang. Timeout serial -> LidarError.
        """
        ser = self._need()
        buf = b""
        current = []
        while True:
            chunk = ser.read(64)
            if not chunk:
                raise LidarError("timeout data scan (cek motor/kabel)")
            buf += chunk
            samples, buf = parse_scan_stream(buf)
            for start, angle, dist, quality in samples:
                if start and current:
                    yield current
                    current = []
                if dist > 0.0:
                    current.append((angle, dist, quality))


# ========================= selftest =========================
def _selftest():
    import struct

    def sample(angle_deg, dist_mm, quality, start):
        b0 = (quality << 2) | (0x02 if start else 0)
        aq = int(round(angle_deg * 64)) & 0xFFFF
        dq = int(round(dist_mm * 4)) & 0xFFFF
        return struct.pack("<BBHH", b0, aq & 0xFF, aq >> 8, 0)[:1] + \
            bytes([aq & 0xFF, aq >> 8, dq & 0xFF, dq >> 8])

    # 2 revolusi sintetis: rev1 3 titik, rev2 3 titik
    rev1 = [sample(0.0, 1000, 15, True),
            sample(90.0, 2000, 14, False),
            sample(180.0, 3000, 13, False)]
    rev2 = [sample(10.0, 1111, 12, True),
            sample(100.0, 2222, 11, False),
            sample(200.0, 0, 0, False)]  # distance0 = invalid
    stream = b"".join(rev1 + rev2)

    # validasi deskriptor respons biasa
    desc = bytes([0x5A, 0x5A, 0x03, 0x00, 0x5A ^ 0x5A ^ 0x03 ^ 0x00])
    assert parse_response_descriptor(desc) == 3

    buf = stream
    got = []
    current = []
    while buf:
        samples, buf = parse_scan_stream(buf)
        for start, angle, dist, quality in samples:
            if start and current:
                got.append(current)
                current = []
                if len(got) == 2:
                    break
            if dist > 0.0:
                current.append((angle, dist, quality))
        if len(got) == 2:
            break
    else:
        if current and got:
            got.append(current)

    assert got[0][0][0] == 0.0 and got[0][0][1] == 1000.0
    assert got[0][1][0] == 90.0 and got[0][1][1] == 2000.0
    assert len(got[1]) == 2  # distance0 dibuang
    assert got[1][0][1] == 1111.0
    print("selftest OK: 2 revolusi terparse,", sum(len(s) for s in got),
          "titik valid")


if __name__ == "__main__":
    _selftest()
