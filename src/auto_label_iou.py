"""Benchmark IoU cho gán nhãn 2D từ LiDAR (topic F).

So sánh 2 cách dựng box 2D từ nhãn 3D, trên nhiều mức lệch calibration yaw:
  - corners3d    : chiếu 8 góc của 3D box GT lên ảnh, lấy min/max -> box 2D.
  - lidar_points : chiếu các điểm LiDAR nằm trong 3D box GT, lấy min/max -> box 2D.
Metric là IoU giữa box gợi ý và box 2D GT trong label_2.
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from starter.datasets import list_frames, load_frame
from starter.projection import box3d_corners_cam, cam_to_image, perturb_extrinsic, velo_to_cam

DEFAULT_CLASSES = ("Car", "Pedestrian", "Cyclist", "Van", "Truck")


def as_homogeneous(points: np.ndarray) -> np.ndarray:
    return np.hstack([points, np.ones((points.shape[0], 1), dtype=np.float64)])


def apply_transform(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    return (as_homogeneous(points) @ matrix.T)[:, :3]


def box3d_mask(points_cam: np.ndarray, obj) -> np.ndarray:
    h, w, l = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    rotation = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    local = (points_cam - obj.location) @ rotation
    return (
        (np.abs(local[:, 0]) <= l / 2)
        & (local[:, 1] >= -h)
        & (local[:, 1] <= 0)
        & (np.abs(local[:, 2]) <= w / 2)
    )


def iou_xyxy(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def box_from_uv(uv: np.ndarray, shape, min_points: int):
    if len(uv) < min_points:
        return None
    h, w = shape[:2]
    box = np.array([uv[:, 0].min(), uv[:, 1].min(), uv[:, 0].max(), uv[:, 1].max()])
    if box[2] - box[0] <= 0 or box[3] - box[1] <= 0:
        return None
    box[[0, 2]] = np.clip(box[[0, 2]], 0, w - 1)
    box[[1, 3]] = np.clip(box[[1, 3]], 0, h - 1)
    return box


def evaluate(data_root, frames, yaw_degrees, classes, min_dist, max_dist) -> list[dict]:
    rows: list[dict] = []
    for frame_id in frames:
        frame = load_frame(data_root, frame_id)
        shape = frame["image"].shape
        p2 = frame["calib"].P2
        transform_true = frame["calib"].T_cam_velo
        points_cam_true = velo_to_cam(frame["points"][:, :3], frame["calib"])

        targets = []
        for obj_index, obj in enumerate(frame["labels"]):
            if obj.type not in classes:
                continue
            dist = float(np.linalg.norm(obj.location))
            if dist < min_dist or dist > max_dist:
                continue
            targets.append((obj_index, obj, dist, box3d_mask(points_cam_true, obj)))

        for yaw in yaw_degrees:
            calib_p = perturb_extrinsic(frame["calib"], yaw_deg=yaw)
            drift = calib_p.T_cam_velo @ np.linalg.inv(transform_true)
            for obj_index, obj, dist, mask_gt in targets:
                corners_uv, _, _ = cam_to_image(apply_transform(drift, box3d_corners_cam(obj)), p2, shape)
                pred_corners = box_from_uv(corners_uv, shape, min_points=4)

                if mask_gt.any():
                    points_drifted = apply_transform(drift, points_cam_true[mask_gt])
                    points_uv, _, _ = cam_to_image(points_drifted, p2, shape)
                else:
                    points_uv = np.empty((0, 2))
                pred_points = box_from_uv(points_uv, shape, min_points=2)

                for method, pred in (("corners3d", pred_corners), ("lidar_points", pred_points)):
                    rows.append({
                        "frame": frame_id,
                        "obj_index": obj_index,
                        "type": obj.type,
                        "dist_m": round(dist, 3),
                        "method": method,
                        "yaw_deg": yaw,
                        "iou": round(iou_xyxy(pred, obj.bbox) if pred is not None else 0.0, 4),
                        "valid": int(pred is not None),
                    })
    return rows


def summarize(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, float], list[float]] = {}
    for row in rows:
        groups.setdefault((row["method"], row["yaw_deg"]), []).append(row["iou"])
    summary = []
    for (method, yaw), ious in sorted(groups.items()):
        values = np.array(ious, dtype=float)
        summary.append({
            "method": method,
            "yaw_deg": yaw,
            "n": int(values.size),
            "mean_iou": round(float(values.mean()), 4),
            "median_iou": round(float(np.median(values)), 4),
            "pct_iou_lt_0.7": round(float((values < 0.7).mean() * 100), 2),
            "pct_iou_ge_0.8": round(float((values >= 0.8).mean() * 100), 2),
        })
    return summary


def write_csv(path, rows, fieldnames) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot(summary, out_path) -> None:
    methods = sorted({row["method"] for row in summary})
    fig, axes = plt.subplots(1, 2, figsize=(11.0, 4.2))
    for method in methods:
        series = sorted((row for row in summary if row["method"] == method), key=lambda row: row["yaw_deg"])
        yaws = [row["yaw_deg"] for row in series]
        axes[0].plot(yaws, [row["mean_iou"] for row in series], marker="o", label=method)
        axes[1].plot(yaws, [row["pct_iou_lt_0.7"] for row in series], marker="o", label=method)
    axes[0].set_xlabel("Do lech yaw (do)")
    axes[0].set_ylabel("IoU trung binh")
    axes[0].set_title("IoU trung binh theo do lech yaw")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_xlabel("Do lech yaw (do)")
    axes[1].set_ylabel("% object co IoU < 0.7")
    axes[1].set_title("Ti le gan co 'can review' theo do lech yaw")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)


def main() -> None:
    parser = argparse.ArgumentParser(description="Benchmark IoU giua box 2D goi y tu LiDAR va box 2D GT (topic F)")
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frames", nargs="*", default=None, help="danh sach frame id; mac dinh lay tat ca")
    parser.add_argument("--yaw-deg", nargs="*", type=float, default=[0.0, 0.5, 1.0, 2.0, 3.0])
    parser.add_argument("--classes", nargs="*", default=list(DEFAULT_CLASSES))
    parser.add_argument("--min-dist", type=float, default=0.0)
    parser.add_argument("--max-dist", type=float, default=1e9)
    parser.add_argument("--seed", type=int, default=0, help="giu cho tai lap; thi nghiem nay tat dinh")
    parser.add_argument("--out-per-object", default="results/auto_label_iou_per_object.csv")
    parser.add_argument("--out-summary", default="results/auto_label_iou_summary.csv")
    parser.add_argument("--out-fig", default="results/figures/auto_label_iou_vs_yaw.png")
    args = parser.parse_args()

    np.random.seed(args.seed)
    frames = args.frames if args.frames else list_frames(args.data_root)
    rows = evaluate(args.data_root, frames, args.yaw_deg, set(args.classes), args.min_dist, args.max_dist)
    summary = summarize(rows)

    write_csv(args.out_per_object, rows, ["frame", "obj_index", "type", "dist_m", "method", "yaw_deg", "iou", "valid"])
    write_csv(args.out_summary, summary, ["method", "yaw_deg", "n", "mean_iou", "median_iou", "pct_iou_lt_0.7", "pct_iou_ge_0.8"])
    plot(summary, args.out_fig)

    print(f"frames={len(frames)} rows={len(rows)}")
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
