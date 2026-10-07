# Báo cáo Day 6: Hỗ trợ gán nhãn 2D bằng LiDAR (topic F)

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Phùng Quang Minh Huy
- **MSSV:** 2A202602610
- **Lớp:** AI20K — Track 4 (Computer Vision and Robotics)
- **Link repo:** https://github.com/huy20/PhungQuangMinhHuy-2A202602610-Track4-Day21
- **Topic:** F — Hỗ trợ gán nhãn bằng LiDAR (auto-label support)
- **Dataset:** data/synthetic (debug code), data/kitti_mini (thí nghiệm chính)
- **Các frame đã dùng:** synthetic 000000; KITTI 000001 (cyclist), 000008 (đông xe), 000009 (xe xa > 50 m), 000011 (nhiều người đi bộ), 000025 (vật rất gần < 6 m), 000049 (nhiều vật bị che khuất)

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

**Claim nháp (CP1):** Trên `data/kitti_mini`, khi dùng calibration gốc, box 2D dựng từ các điểm LiDAR nằm trong 3D box GT đạt **IoU trung bình ≥ 0.80** so với box 2D GT cho `Car`/`Pedestrian` ở dải 5–30 m; chỉ lệch yaw **1°** đã làm **IoU trung bình giảm ≥ 0.10** và đẩy **tỉ lệ object có IoU < 0.7 lên trên 30%**, đủ để dùng **ngưỡng IoU 0.7** làm cờ "label cần review".

**Diễn giải đo gì – trên frame nào – với mức nào:**
- **Đo:** IoU giữa box 2D gợi ý và box 2D GT (`label_2`), theo từng object và trung bình mỗi cấu hình.
- **So sánh 2 cách:** (A) chiếu 8 góc 3D box (`box3d_corners_cam`) vs (B) min/max điểm LiDAR đã chiếu nằm trong 3D box.
- **Mức thay đổi:** calibration yaw ∈ {0°, 0.5°, 1°, 2°, 3°} (mỗi lần chỉ đổi 1 yếu tố), cộng thêm dịch `tx/ty/tz` = 5 cm ở bước sau.
- **Frames:** 000001, 000008, 000009, 000011, 000025, 000049; debug trên `data/synthetic/000000`.

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Ghi rõ đường dẫn file trong `results/`.

| Cấu hình / mức perturb | Metric 1 | Metric 2 | Ghi chú |
|---|---|---|---|
| [ĐIỀN] | | | |

![demo](../results/figures/[ĐIỀN].png)

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch.

```bash
# CP2 - demo đầu tiên (sau khi đã viết 2 hàm TODO(CP2) trong starter/projection.py)
python -m starter.projection --data-root data/synthetic --frame 000000
python -m starter.projection --data-root data/kitti_mini --frame 000011
python -m starter.projection --data-root data/nuscenes_mini_subset --frame scene-0103_010
# ảnh overlay lưu vào results/figures/overlay_<frame>_*.png
```

## 6. Khai báo sử dụng AI

Ghi rõ đã dùng công cụ AI nào, dùng vào việc gì, và bạn đã tự kiểm chứng kết quả đó bằng cách nào. Nếu không dùng AI, ghi "Không sử dụng". Xem quy định ở `RULES.md` mục 2.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| [ĐIỀN] | | |
