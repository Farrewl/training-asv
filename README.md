# training asv — repo riset, test hardware & dataset

Repo ini khusus untuk **coba-coba, riset dan persiapan lomba** (bukan kode lomba). Siap dipakai untuk: test thruster/servo, test koneksi MAVLink, test kamera + capture dataset YOLO, test lidar + deteksi bentuk (RPLIDAR-like), dan training lokal.

## Cepat mulai

Siapkan `.venv` (sudah ada: lihat requirements.txt).

```bash
# Linux/macOS/Git Bash
.venv/bin/pip install -r requirements.txt  # jika belum

# Windows
.venv\Scripts\pip install -r requirements.txt
```

Training YOLO (opsional, berat): `.venv/bin/pip install -r requirements-train.txt`

## Tool

| Tool | Fungsi |
|---|---|
| `tools/conn_check.py` | Scan port, heartbeat Pixhawk, RC channels, gamepad (opsional) |
| `tools/thruster_test.py` | Uji thruster via DO_SET_SERVO atau RC_OVERRIDE. Timer netral otomatis, ada mode dry-run (`--selftest`) |
| `tools/servo_test.py` | Sweep/manual servo per kanal (DO_SET_SERVO) |
| `tools/camera_capture.py` | Preview + capture → auto split train/val + `data.yaml` untuk YOLO |
| `tools/lidar_test.py` | Live plot 2D / raw hex dump / sim tanpa hardware |
| `tools/lidar_shapes.py` | Cluster + detect buoy & gate dari scan lidar (sim/live) |

Contoh singkat:
```bash
python tools/conn_check.py
python tools/thruster_test.py --selftest
python tools/lidar_shapes.py --sim
python tools/camera_capture.py --cam 0 --dir dataset --classes buoy
```

## Lidar (RPLIDAR A1/A2/A3 kompatibel, deteksi umum)

Driver minimal ada: `lidar/rplidar.py` (motor via DTR/RTS, deskriptor scan, parser stream 5-byte). Dipakai oleh `lidar_test.py` & `lidar_shapes.py`.

- **Merek tak pasti**? Jalankan: `python tools/lidar_test.py --raw --port COM5 --n 60` → dump sample hex, cocokkan pola 5-byte atau kirimkan output ke sesi ini.
- Algoritma bentuk: cluster (gap antar-titik) → Kasa circle-fit → klasifikasi `buoy`/`obstacle` → pair menjadi `gate` (baseline dalam range).

## Training + ekspor ONNX

Dua notebook:

| Notebook | Isi |
|---|---|
| `notebooks/01_train_yolo.ipynb` | Training YOLOv8 → validasi → ekspor ONNX (opset 12, simplify=True) agar kompatibel `cv2.dnn` (Repo A/Repo B). Bisa pakai dataset **lokal** (hasil capture) atau **unduh dari Roboflow** (opsional, API key lewat env) |
| `notebooks/02_lidar_shape_sim.ipynb` | Riset lidar: simulasi scan → cluster → buoy/gate, tanpa hardware |

**`.pt` vs `.onnx`:** training otomatis menghasilkan `runs/detect/train/weights/best.pt` (model asli, untuk Python/ultralytics). Kapal memakai `cv2.dnn` yang **tidak bisa membaca `.pt`**, karena itu `best.pt` diekspor ke `.onnx` (isi identik, format beda). Butuh `.pt` saja? Lewati cell ekspor ONNX.

Alur: capture (`tools/camera_capture.py`) atau Roboflow → label → `01_train_yolo.ipynb` → `scripts/export_onnx.py`. Dataset lokal: lihat `dataset/README.md`.

Ekspor ONNX dari terminal (sama seperti cell di notebook):

```bash
python scripts/export_onnx.py --input runs/detect/train/weights/best.pt --output weights/best.onnx
```

### Dataset dari Roboflow (opsional)

Kalau dataset sudah dianotasi di Roboflow, isi `RF_WORKSPACE` / `RF_PROJECT` / `RF_VERSION`
(lihat URL `app.roboflow.com/<workspace>/<project>/<versi>`) lalu set `USE_ROBOFLOW = True`
di `notebooks/01_train_yolo.ipynb`. API key **jangan** ditulis di notebook:

```bash
export ROBOFLOW_API_KEY="xxxx"          # Linux/macOS
setx ROBOFLOW_API_KEY "xxxx"            # Windows (restart terminal setelahnya)
```

Alternatif tanpa SDK: dashboard Roboflow → *Download Dataset* → format **YOLOv8** → ekstrak zip ke `dataset/`.

## Catatan safety

- **Thruster test**: lepas baling-baling, pakai `--selftest` dulu, selalu kembali netral. Jangan test di air tanpa pengawasan.
- **Servo/ESC**: DO_SET_SERVO memaksa output; pastikan channel & PWM aman (1000–2000, batas netral).
- **Lidar motor**: sebagian adapter perlu DTR/RTS dinyalakan. Kalau muncul timeout scan, cek health + kabel.

## Strukturnya

```text
training asv/
├── dataset/     README + data.yaml hasil capture
├── lidar/       rplidar driver + shapes (cluster/circle-fit/gate)
├── notebooks/   01_train_yolo.ipynb, 02_lidar_shape_sim.ipynb
├── scripts/     export_onnx.py
├── tools/       conn_check, thruster/servo, camera_capture, lidar_test, lidar_shapes
├── requirements.txt
├── requirements-train.txt
└── README.md
```
