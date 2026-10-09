# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

## Thông tin học viên

- Họ tên: Nguyễn Đức Anh
- MSSV: 2A202602625
- Email: nguyenanh10a2cvp@gmail.com
- Link repo (fork): https://github.com/Munfond/K4-Track4-Day23-Sensor-Fusion-Student
- Commit hash nộp (`git rev-parse HEAD`): Lấy hash 40 ký tự của commit CP6 chứa báo cáo này bằng `git rev-parse HEAD` khi nộp LMS. Không ghi hash của chính commit vào nội dung commit đó.

Repo hiện tại là fork Public. Tên repo theo mẫu cần dùng khi nộp là
`K4-L2L3-DAY23-NguyenDucAnh-2A202602625-SensorFusion`; học viên tự đổi tên repo,
cập nhật link nộp và nộp GitHub/VLearn.

## Tóm tắt kết quả

Nguồn số liệu: [metrics.json](artifacts/metrics.json) và
[grade_run.log](artifacts/grade_run.log), sinh sau khi hoàn thành Part E–H.

- `fusion_mode`: `compare`.
- `frames`: `[0, 198]`, tính cả hai đầu, 199 frame mỗi mode.
- `segment`: `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`.
- `seed`: `0`.
- `detection.precision`: `0.9700934579439252`.
- `detection.recall`: `0.7004048582995951`.
- `detection.tp/fp/fn`: `519 / 16 / 222`.

| Chỉ số tracking | LiDAR | LiDAR + camera |
|---|---:|---:|
| `rmse` (m) | 0.15032268781360134 | 0.1358667883353908 |
| `matches` | 502 | 502 |
| `sum_sq_err` (m²) | 11.343649056695735 | 9.26681165463209 |
| `ghost_track_frames` | 0 | 0 |
| `missed_gt_frames` | 239 | 239 |
| `mean_confirmed_tracks` | 2.522613065326633 | 2.522613065326633 |
| `precision_track` | 1.0 | 1.0 |
| `coverage` | 0.9672447013487476 | 0.9672447013487476 |

Lệnh chạy chấm điểm từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

RMSE là `sqrt(sum_sq_err / matches)` trên sai số vị trí 3D của confirmed tracks
ghép một-một với nhãn xe hợp lệ trong cổng XY 2 m. Hai mode có cùng 502 cặp ghép,
0 ghost và 239 miss. Fused giảm RMSE khoảng `0.01445589947821054 m` (1.45 cm,
khoảng 9.62%) trong lần chạy này; giảm sai số không đi kèm giảm số cặp được ghép.
`precision_track = 502 / (502 + 0) = 1`,
`coverage = 502 / 519 ≈ 0.9672447013`. Cả hai mode đạt RMSE ≤ 0.45 m,
precision_track ≥ 0.75 và coverage ≥ 0.70, nên hệ số chất lượng `q = 1`.
`rmse_fused - rmse_lidar ≈ -0.0144558995 m ≤ 0.05 m`, đạt điều kiện nhất quán.

239 miss là tổng theo frame, không phải 239 xe khác nhau. Tổng GT hợp lệ là
`519 + 222 = 741`, và `502 + 239 = 741`. Coverage cao không có nghĩa là theo dõi
được mọi GT: mẫu số của coverage là detection TP, trong khi detector còn 222 FN.
Ở frame 0–3, log ghi `confirmed = 0`; frame 4 mới có hai confirmed tracks,
phù hợp giai đoạn tích lũy score ban đầu. Ví dụ ở frame 46, hai mode đều có
2 matches, 0 ghost và 1 miss, nhưng `sum_sq_err` giảm từ
`0.04450990452758851` ở LiDAR xuống `0.04033337252935057` ở fused.

Đã đối chiếu đủ sáu file: metrics/log tổng và metrics/log riêng từng mode.
Log tổng có 398 record, mỗi `(mode, frame)` đúng một record, 199 record mỗi mode.
Mỗi record thỏa `matches + ghosts == confirmed` và
`matches + misses == valid_gt`; tổng, RMSE và trung bình tính lại từ log khớp metrics.
Detection của hai mode giống nhau.

Camera trong lab lấy tâm hộp 2D ground-truth FRONT rồi thêm nhiễu theo seed,
không chạy detector ảnh. Kết quả trên chứng minh tác dụng của camera update với
đo mô phỏng trong segment này, không chứng minh chất lượng camera detector hay
khả năng tổng quát của hệ thống perception độc lập với ground truth.

## Giải thích ngắn (Parts E–H)

1. **Khác biệt đo LiDAR 3D và camera 2D trong EKF (`z`, `R`).**
   State là `[px, py, pz, vx, vy, vz]ᵀ`. LiDAR đo tâm hộp 3D trong hệ cảm biến,
   `z` có shape 3×1 và `R` có shape 3×3, đơn vị m². Camera đo pixel `[u, v]ᵀ`,
   `z` có shape 2×1 và `R = diag(sigma_cam_i², sigma_cam_j²)`, đơn vị pixel²;
   với tham số hiện tại là `diag(25, 25)`. Mô hình LiDAR tuyến tính theo vị trí;
   camera đổi `p_s = R_rotation p + t`, rồi chiếu
   `u = c_i - f_i y_s/x_s`, `v = c_j - f_j z_s/x_s`. EKF dùng Jacobian camera
   từ platform. Xem [camera_fusion.py](workspace/camera_fusion.py),
   [kalman.py](workspace/kalman.py) và
   [Sensor.get_hx/get_H](../platform/fusion_lab/tracking/sensors.py).

2. **Vì sao cần gating Mahalanobis trước khi gán?**
   `gamma = z - h(x)`, `S = H P Hᵀ + R`; `d² = gammaᵀ S⁻¹ gamma` đo residual
   theo độ bất định của cả track và sensor. Khoảng cách Euclid bỏ qua độ bất định
   này và không so sánh trực tiếp được mét với pixel. Gating χ² dùng
   `chi2.ppf(gating_threshold, sensor.dim_meas)`, với chiều đo 3 cho LiDAR và 2
   cho camera; cặp bị loại nhận cost vô hạn trước khi greedy chọn cặp nhỏ nhất.
   Kiểm tra FOV trước Mahalanobis để điểm sau camera không bị chiếu.
   Xem `mahalanobis_distance`, `chi2_gate`, `association_cost_matrix` và
   `pick_next_pair` trong [association.py](workspace/association.py).

3. **Track-then-fuse hay fuse-then-track?**
   Đây là track-then-fuse: một tracker giữ danh sách track chung. Mỗi frame
   predict một lần, gán/update LiDAR và quản lý vòng đời, rồi gán/update camera
   trên các track đó nếu có nhãn FRONT. Không predict thêm giữa hai sensor,
   không ghép đo thành một input chung trước tracking. Thứ tự được thể hiện
   trực tiếp ở [run_lab.py, vòng lặp frame](../platform/fusion_lab/scripts/run_lab.py#L198-L209).
   `grade_run.log` chỉ ghi kết quả đánh giá cuối frame, không ghi từng bước
   predict/update; các record `mode = lidar/fused` chứng minh hai lượt chạy
   trên cùng frame. Ở frame 46, số ghép giữ nguyên nhưng sai số fused nhỏ hơn,
   là một ví dụ kết quả sau camera update, không phải log nội bộ thứ tự update.

4. **Camera lệch calibration gây triệu chứng gì?**
   Sai extrinsic/intrinsic làm `h(x)` sai có hệ thống, nên innovation
   `z - h(x)` có thể lệch kéo dài theo hướng hoặc tăng độ lớn. Sai số có thể
   phụ thuộc vị trí và khoảng cách xe. Nếu `d²` vượt cổng χ², camera update bị
   loại; nếu vẫn trong cổng, update có thể kéo state sai hướng và làm tăng RMSE.
   Rotation còn ảnh hưởng FOV và Jacobian. Đây là phân tích từ công thức trong
   [camera_fusion.py](workspace/camera_fusion.py) và
   [association.py](workspace/association.py); bài này chưa chạy thí nghiệm
   lệch calibration, nên không có số liệu thực nghiệm về calibration drift.

5. **Vì sao cần `sensor` tường minh ở frame rỗng?**
   Khi `meas_list` rỗng không thể suy sensor từ `meas_list[0]`. Vẫn phải gọi
   `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)` để lượt
   LiDAR trừ score của track miss trong FOV và xét xóa. Nếu là lượt camera,
   manager không thay đổi vòng đời. Cặp đã ghép gọi `filter_obj.update` rồi
   `handle_updated_track(track, sensor)`; manager chỉ ghi hit khi sensor là
   LiDAR. Đo LiDAR chưa ghép mới có thể tạo track. Camera chỉ EKF update,
   không score/init/delete. Xem [associate_and_update](workspace/association.py)
   và [TrackManager](../platform/fusion_lab/tracking/manager.py).
   Các test empty-pass, camera-hit và camera-empty/unmatched trong
   [test_tracking_regressions.py](tests/test_tracking_regressions.py) đã pass.

6. **Điều kiện xác nhận, giữ confirmed và xóa track.**
   Khởi tạo từ LiDAR: vị trí đổi về hệ xe, vận tốc bằng 0; covariance vị trí
   là `R_rotation R_measurement R_rotationᵀ`, covariance vận tốc lấy từ
   `sigma_p44/55/66²`. Score đầu là `1/window`, trạng thái `initialized`.
   Hit cộng `1/window`, tối đa 1; hit chưa đủ xác nhận chuyển thành `tentative`.
   Khi score **lớn hơn** `confirmed_threshold` thì `confirmed`. Miss trong FOV
   trừ `1/window`, nhưng không hạ trạng thái đã confirmed. Xóa nếu `Pxx` hoặc
   `Pyy` **lớn hơn** `max_P`, hoặc confirmed có score **nhỏ hơn**
   `delete_threshold`, hoặc chưa confirmed có score **≤ 0**. Với tham số hiện
   tại: `window = 6`, `confirmed_threshold = 0.8`, `delete_threshold = 0.6`,
   `max_P = 9 m²`. Các dấu so sánh ở biên phải được giữ đúng.
   Xem [track_management.py](workspace/track_management.py) và test score/
   deletion boundaries trong [test_tracking_contract.py](tests/test_tracking_contract.py).

## Bonus (không bắt buộc)

- Không. Chưa thực hiện export CVAT, trực quan hóa bonus hoặc thí nghiệm calibration.

## Khai báo sử dụng AI (bắt buộc)

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): OpenAI Codex trong ứng dụng desktop.
- Dùng cho phần nào (hàm, câu hỏi, debug): Codex đọc tài liệu và tóm tắt yêu cầu; chuẩn bị Python 3.12, dữ liệu và weights; cài các hàm Part E–H; xử lý encoding UTF-8 trên Windows; chạy test và Waymo; đối chiếu log/metrics; soạn nội dung kết quả và sáu câu giải thích trong báo cáo này; tạo commit theo checkpoint. Không dùng AI tạo số liệu giả hoặc sửa artifacts bằng tay.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Các kiểm tra tự động được thực hiện trong phiên Codex: CP1 có 4 test passed; CP2 có 20 test passed; CP3 có 9 test passed; CP4 toàn bộ 128 test passed, không failed/xfailed. Kiểm tra thêm defaults/overrides của `dt`, `q`, ndarray và đầu vào không bị sửa; khởi tạo LiDAR có rotation/translation. Chạy segment mặc định frame 0–198 bằng `compare --seed 0`; tính lại metrics từ 398 record, kiểm tra invariant và đối chiếu cả sáu artifact. Học viên cần tự đọc và giải thích được code cùng nội dung báo cáo khi vấn đáp.

## Checklist nộp

- [x] Part E–H đã implement; toàn bộ 128 test passed, không failed/xfailed.
- [x] Giữ nguyên Part A–D và platform.
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, frame 0–198.
- [x] Sáu file artifacts đã commit, không sửa tay.
- [x] Đã điền họ tên, MSSV, email, kết quả, sáu câu giải thích và khai báo AI.
- [x] Không commit dữ liệu Waymo, weights, `.env`, `paths.yaml`, file nén hoặc API key.
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`.
- [ ] Đổi tên repo theo mẫu và cập nhật link nộp nếu đổi tên.
- [ ] Push commit cuối, kiểm tra hash trên GitHub và nộp URL/hash trên VLearn (học viên tự thực hiện).
