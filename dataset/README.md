# Dataset untuk pelatihan YOLOv8 (lokal)

## Struktur hasil camera_capture.py
```text
dataset/
├── images/train/, images/val/
├── labels/train/, labels/val/
└── data.yaml
```

## Format label YOLO
Setiap `labels/*.txt` berisi:
```text
<class> <x> <y> <w> <h>
```
Nilai **dinormalisasi**: 0..1. 
- x, y = pusat bounding box
- w, h = lebar/tinggi relatif terhadap ukuran image

## Workflow singkat
1. `python tools/camera_capture.py --cam 0` → tekan `s` simpan frame (80% train, 20% val). Tekan `q` keluar.
2. Label pakai [labelImg](https://github.com/tzutalin/labelImg) (format YOLO).
3. Train: buka `notebooks/01_train_yolo.ipynb` → arahkan path ke `dataset/data.yaml`.
4. Ekspor ONNX: `python scripts/export_onnx.py --input runs/detect/train/weights/best.pt --output weights/best.onnx`
5. Copy `weights/best.onnx` ke repo lomba (Repo A/Repo B) di folder `weights/`.

## Tips
- Ambil varian cahaya berbeda (siang/redup) & sudut berbeda → generalisasi lebih kuat.
- Hindari capture terburuk (blur/motion). Undo: `d` di `camera_capture`.
- Class name sebaiknya sama antar sesi. Bisa di-edit `--classes buoy,gate,...`.
