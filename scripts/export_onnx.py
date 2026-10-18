"""scripts/export_onnx.py — ekspor weights/best.pt -> best.onnx kompatibel cv2.dnn.

Asumsi: best.pt di runs/detect/train/weights/best.pt (hasil notebook 01).

OPSET 12 (cv2.dnn lebih aman). Opsional --dynamic untuk batch=1..N.
Tidak butuh torch? ultralytics punya export. Kalau torch belum ada,
warning dari ultralytics muncul, tapi bisa gunakan .venv training terpisah.

Pakai:
  python scripts/export_onnx.py --input runs/detect/train/weights/best.pt
  python scripts/export_onnx.py --input weights/best.pt --output weights/best.onnx --opset 12
"""

import argparse
import os
import sys

try:
    from ultralytics import YOLO
except ImportError:
    print("ultralytics tidak terpasang. Instal: .venv/bin/pip install -r requirements-train.txt")
    sys.exit(1)

try:
    import torch
    print("torch OK, device:", torch.device("cuda" if torch.cuda.is_available() else "cpu"))
except ImportError:
    print("torch belum terpasang — ultralytics tetap bisa export, tapi lebih aman pakai torch.")


def main():
    ap = argparse.ArgumentParser(description="Export best.pt -> ONNX")
    ap.add_argument("--input", required=True)
    ap.add_argument("--output")
    ap.add_argument("--opset", type=int, default=12)
    ap.add_argument("--dynamic", action="store_true")
    args = ap.parse_args()
    if not os.path.exists(args.input):
        ap.error("%s tidak ditemukan" % args.input)
    model = YOLO(args.input)
    out = model.export(format="onnx", opset=args.opset,
                       simplify=True, dynamic=args.dynamic)
    if not out:
        print("export gagal")
        sys.exit(1)
    print("OK:", out)
    if args.output:
        import shutil
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        shutil.copy(out, args.output)
        print("disalin ke:", args.output)


if __name__ == "__main__":
    main()
