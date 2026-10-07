"""Ve anh failure case cho topic F: GT box, box goi y tu 8 goc 3D (A) va tu diem LiDAR (B)."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from auto_label_iou import apply_transform, box3d_mask, box_from_uv, iou_xyxy
from starter.datasets import load_frame
from starter.projection import box3d_corners_cam, cam_to_image, perturb_extrinsic, velo_to_cam


def project_raw(points_cam, p2):
    homogeneous = np.hstack([points_cam, np.ones((points_cam.shape[0], 1), dtype=np.float64)])
    projected = homogeneous @ np.asarray(p2, dtype=np.float64).T
    return projected[:, :2] / projected[:, 2:3], points_cam[:, 2]


def make_figure(data_root, frame_id, obj_index, out_path, yaw_deg=0.0) -> None:
    frame = load_frame(data_root, frame_id)
    image = frame["image"].copy()
    h, w = image.shape[:2]
    shape = image.shape
    p2 = frame["calib"].P2
    obj = frame["labels"][obj_index]

    transform_true = frame["calib"].T_cam_velo
    calib_p = perturb_extrinsic(frame["calib"], yaw_deg=yaw_deg)
    drift = calib_p.T_cam_velo @ np.linalg.inv(transform_true)

    corners = apply_transform(drift, box3d_corners_cam(obj))
    points_cam_true = velo_to_cam(frame["points"][:, :3], frame["calib"])
    points_drifted = apply_transform(drift, points_cam_true)

    gx1, gy1, gx2, gy2 = (int(round(v)) for v in obj.bbox)
    cv2.rectangle(image, (gx1, gy1), (gx2, gy2), (0, 255, 0), 2)
    cv2.putText(image, f"GT {obj.type}", (gx1, max(12, gy1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    uv_c, _, mask_c = cam_to_image(corners, p2, shape)
    uv_raw, z_raw = project_raw(corners, p2)
    n_inside = int(mask_c.sum())
    if n_inside >= 4:
        box_a = box_from_uv(uv_c, shape, 4)
        ax1, ay1, ax2, ay2 = (int(round(v)) for v in box_a)
        cv2.rectangle(image, (ax1, ay1), (ax2, ay2), (0, 165, 255), 2)
        cv2.putText(image, "A: 8 goc 3D box", (ax1, max(12, ay1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        iou_a = iou_xyxy(box_a, obj.bbox)
    else:
        iou_a = 0.0
        for (u, v), z in zip(uv_raw, z_raw):
            if z <= 0.1:
                continue
            cv2.circle(image, (int(np.clip(u, 0, w - 1)), int(np.clip(v, 0, h - 1))), 6, (0, 0, 255), 2)
        cv2.putText(image, f"A: DROPPED, chi {n_inside}/8 goc nam trong anh", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

    selected = box3d_mask(points_cam_true, obj)
    n_points = int(selected.sum())
    box_b = None
    if n_points >= 2:
        uv_p, _, _ = cam_to_image(points_drifted[selected], p2, shape)
        box_b = box_from_uv(uv_p, shape, 2)
    if box_b is not None:
        bx1, by1, bx2, by2 = (int(round(v)) for v in box_b)
        cv2.rectangle(image, (bx1, by1), (bx2, by2), (255, 0, 0), 2)
        cv2.putText(image, "B: diem LiDAR", (bx1, max(12, by1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        iou_b = iou_xyxy(box_b, obj.bbox)
    else:
        iou_b = 0.0
        cv2.putText(image, f"B: DROPPED, chi {n_points} diem LiDAR trong 3D box", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

    dist = float(np.linalg.norm(obj.location))
    cv2.putText(image, f"{frame_id} obj{obj_index} {obj.type} dist={dist:.1f}m yaw={yaw_deg} IoU A={iou_a:.2f} B={iou_b:.2f}",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    out = Path(out_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image)
    print(f"{out}: corners_inside={n_inside}/8 lidar_points={n_points} IoU_A={iou_a:.3f} IoU_B={iou_b:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Ve anh failure case cho topic F")
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frame", required=True)
    parser.add_argument("--obj-index", type=int, required=True)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    make_figure(args.data_root, args.frame, args.obj_index, args.out, args.yaw_deg)


if __name__ == "__main__":
    main()
