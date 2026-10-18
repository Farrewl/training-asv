"""tools/camera_capture.py — preview + capture dataset untuk YOLO.

Mode: live preview (imutils/Qt tidak perlu, murni OpenCV). Tekan:
  s  = simpan frame + buat label kosong template (YOLO txt)
  d  = hapus frame terakhir (undo)
  q/ESC = keluar

Struktur dataset:
  dataset/
    images/train/, images/val/
    labels/train/, labels/val/
    data.yaml

Split default 80/20 train/val (acak). Label YOLO (class x y w h normalisasi)
dibuat kosong (class 0) jika belum di-label — cocok pakai labelImg/Roboflow.

Pakai:
  python tools/camera_capture.py --cam 0
  python tools/camera_capture.py --cam 1 --width 848 --height 480 --fps 30
  python tools/camera_capture.py --dir dataset --split 0.8 --class-name buoy
"""

import argparse
import os
import random
import shutil
from datetime import datetime

import cv2
import numpy as np

CLASSES = ["object"]


def make_dirs(root):
    for p in (f"{root}/images/train", f"{root}/images/val",
              f"{root}/labels/train", f"{root}/labels/val"):
        os.makedirs(p, exist_ok=True)


def write_yaml(root, class_names):
    yaml_path = os.path.join(root, "data.yaml")
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write("path: %s\n" % os.path.abspath(root))
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("nc: %d\n" % len(class_names))
        f.write("names: %s\n" % class_names)
    return yaml_path


def pick_split():
    return "train" if random.random() < 0.8 else "val"


def save_frame(root, split, frame, class_names):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    base = f"frame_{ts}"
    img_path = os.path.join(root, "images", split, base + ".jpg")
    lbl_path = os.path.join(root, "labels", split, base + ".txt")
    cv2.imwrite(img_path, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    with open(lbl_path, "w", encoding="utf-8") as f:
        for c in class_names:
            pass  # kosong — label manual
    return img_path, lbl_path


def undo_last(root):
    # cari .jpg terbaru di images/train+val
    last = None
    for sp in ("train", "val"):
        d = os.path.join(root, "images", sp)
        if not os.path.exists(d):
            continue
        files = [os.path.join(d, x) for x in os.listdir(d)
                 if x.endswith(".jpg")]
        if not files:
            continue
        cur = max(files, key=os.path.getmtime)
        if last is None or os.path.getmtime(cur) > os.path.getmtime(last):
            last = cur
    if not last:
        return False
    lbl = os.path.splitext(last)[0].replace("images", "labels") + ".txt"
    try:
        os.remove(last)
    except FileNotFoundError:
        pass
    try:
        os.remove(lbl)
    except FileNotFoundError:
        pass
    return True


def open_cam(index, w, h, fps):
    cap = cv2.VideoCapture(index)
    if w:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    if h:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def main():
    ap = argparse.ArgumentParser(description="Capture dataset YOLO")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--dir", default="dataset")
    ap.add_argument("--width", type=int)
    ap.add_argument("--height", type=int)
    ap.add_argument("--fps", type=int)
    ap.add_argument("--split", type=float, default=0.80,
                    help="train ratio (default 0.80)")
    ap.add_argument("--classes", default="object",
                    help="nama kelas dipisah koma (default: object)")
    args = ap.parse_args()

    class_names = [c.strip() for c in args.classes.split(",") if c.strip()]
    if not class_names:
        class_names = CLASSES

    make_dirs(args.dir)
    yaml_p = write_yaml(args.dir, class_names)
    print("Dataset:", os.path.abspath(args.dir))
    print("data.yaml:", yaml_p)
    print("Kelas:", class_names)

    global pick_split
    orig = pick_split

    def pick_split_wrapped():
        if random.random() < args.split:
            return "train"
        return "val"

    pick_split = pick_split_wrapped  # wrap ratio

    cap = open_cam(args.cam, args.width, args.height, args.fps)
    if not cap.isOpened():
        print("Gagal buka kamera index", args.cam)
        return
    win = "camera_capture (s=save d=undo q/ESC)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    saved = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            h, w = frame.shape[:2]
            cv2.putText(frame, f"saved={saved}  s:save  d:undo  q/ESC",
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)
            cv2.imshow(win, frame)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('s'):
                sp = pick_split()
                imgp, lblp = save_frame(args.dir, sp, frame, class_names)
                saved += 1
                print(f"[SAVE] {sp}: {imgp}  (+label {lblp})")
            if k == ord('d'):
                if undo_last(args.dir):
                    saved = max(0, saved - 1)
                    print("[UNDO] frame terakhir dihapus")
                else:
                    print("[UNDO] tidak ada frame")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Selesai. Total tersimpan:", saved)


if __name__ == "__main__":
    main()
