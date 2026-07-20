#!/usr/bin/env python3
"""
Detect AprilTags in the captured camera frames and show the results.

For each image in the input directory, detects AprilTags with OpenCV's
aruco module and draws each tag with 2 px lines using the tag's lower-left
corner as the origin: the x-axis edge in red, the y-axis edge in green,
and the other two sides in yellow. The image is dimmed to 33% so the
annotations stand out. The annotated image is saved to the
output directory and shown in a window. Detection data (tag IDs, corners,
centers) for captures whose filename starts with a timestamp prefix
(e.g. 20260720_082526_*) is written to <prefix>_detections.json.

Keys: any key = next image, q/Esc = quit.

Requires: pip install opencv-contrib-python numpy

Usage:
  python detect_apriltags.py
  python detect_apriltags.py --dir captures --out annotated --json-dir detections
"""

import argparse
import json
import re
from pathlib import Path

import cv2
import numpy as np

FAMILIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}

WINDOW = "apriltags"


def detect_tags(gray, family):
    """Return (corners, ids) for all AprilTags of the given family."""
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return [], np.empty(0, dtype=int)
    return corners, ids.flatten()


RED = (0, 0, 255)  # BGR
GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)


def draw_tags(image, corners, ids):
    """Draw each tag with 2 px edges from its lower-left origin: x-axis red,
    y-axis green, other two sides yellow. Also marks the origin and the ID."""
    scale = image.shape[1] / 1920  # overlay size relative to a 1920px-wide image
    # minimums keep the origin dot and label visible on small (e.g. 384px) frames
    dot_r = max(3, round(5 * scale))
    font_scale = max(0.5, 1.2 * scale)
    label_dy = max(5, round(10 * scale))
    for tag_corners, tag_id in zip(corners, ids):
        # cv2.aruco returns corners in the tag's own frame, clockwise from
        # the top-left: [top-left, top-right, bottom-right, bottom-left].
        tl, tr, br, bl = tag_corners[0].astype(np.int32)
        cv2.line(image, tuple(bl), tuple(br), RED, 2)  # x-axis (bottom edge)
        cv2.line(image, tuple(bl), tuple(tl), GREEN, 2)  # y-axis (left edge)
        cv2.line(image, tuple(tl), tuple(tr), YELLOW, 2)  # top edge
        cv2.line(image, tuple(tr), tuple(br), YELLOW, 2)  # right edge
        cv2.circle(image, tuple(bl), dot_r, RED, -1)  # origin
        top = min((tl, tr, br, bl), key=lambda pt: pt[1])  # topmost corner
        cv2.putText(
            image,
            f"ID {tag_id}",
            (top[0], top[1] - label_dy),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            GREEN,
            2,
        )
    return image


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--dir", default="captures", help="directory with images (default: captures)"
    )
    parser.add_argument(
        "--family",
        default="tag36h11",
        choices=FAMILIES,
        help="AprilTag family (default: tag36h11)",
    )
    parser.add_argument(
        "--out",
        default="annotated",
        help="directory for annotated images (default: annotated)",
    )
    parser.add_argument(
        "--json-dir",
        default="detections",
        help="directory for detection JSON files (default: detections)",
    )
    args = parser.parse_args()

    paths = sorted(
        p for p in Path(args.dir).iterdir() if p.suffix.lower() in IMAGE_EXTS
    )
    if not paths:
        parser.error(f"no images found in {args.dir}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    json_dir = Path(args.json_dir)
    json_dir.mkdir(parents=True, exist_ok=True)

    # Pass 1: detect, annotate, and save everything, so an early quit in the
    # UI pass can't lose data. Detection records are grouped by the capture
    # batch's timestamp prefix (filenames like 20260720_082526_anchor0.jpg).
    batches = {}  # timestamp prefix -> {image name: {"tags": [...]}}
    annotated = []  # (name, image) for the UI pass
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f"{path.name}: unreadable, skipped")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = detect_tags(gray, args.family)
        print(f"{path.name}: {len(ids)} tag(s)", sorted(ids.tolist()))
        image = cv2.convertScaleAbs(image, alpha=0.33, beta=0)  # dim underlay
        draw_tags(image, corners, ids)
        out_path = out_dir / path.name
        cv2.imwrite(str(out_path), image)
        print(f"  saved {out_path}")
        annotated.append((path.name, image))

        match = re.match(r"(\d{8}_\d{6})_", path.name)
        if match:
            tags = sorted(
                (
                    {
                        "id": int(tag_id),
                        "corners": [
                            [round(float(v), 1) for v in pt]
                            for pt in tag_corners[0]
                        ],
                        "center": [
                            round(float(v), 1)
                            for v in tag_corners[0].mean(axis=0)
                        ],
                    }
                    for tag_corners, tag_id in zip(corners, ids)
                ),
                key=lambda t: t["id"],
            )
            batches.setdefault(match.group(1), {})[path.name] = {"tags": tags}

    for prefix, images in batches.items():
        doc = {
            "family": args.family,
            "corner_order": ["top_left", "top_right", "bottom_right", "bottom_left"],
            "images": images,
        }
        json_path = json_dir / f"{prefix}_detections.json"
        json_path.write_text(json.dumps(doc, indent=2) + "\n")
        print(f"saved {json_path}")

    # Pass 2: show the annotated images.
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1440, 810)

    for name, image in annotated:
        cv2.imshow(WINDOW, image)
        key = cv2.waitKey(0) & 0xFF
        if key in (ord("q"), 27):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
