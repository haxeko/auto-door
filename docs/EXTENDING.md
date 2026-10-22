# Mở rộng sang cửa chạy động cơ và khóa điện

Chỉ profile **rèm** (Window Covering) được triển khai. Tài liệu này mô tả điểm mở
rộng cho hai loại thiết bị còn lại. Chưa triển khai vì chưa có thông tin phần cứng
và yêu cầu an toàn cụ thể.

## 1. Nguyên tắc: mỗi loại thiết bị một Matter device type

| Thiết bị | Device type (esp-matter release/v1.5) | Cluster chính | Ghi chú |
|---|---|---|---|
| Rèm cuốn / rèm kéo | Window Covering `0x0202` (`endpoint::window_covering`) | Window Covering (Lift + PositionAwareLift) | **Đã triển khai** |
| Cửa / cổng chạy động cơ | Closure `0x0230` (`endpoint::closure`), Closure Panel `0x0231` (`endpoint::closure_panel`) | Closure Control, Closure Dimension | Device type mới của Matter 1.5, có trong data model esp-matter release/v1.5. Mức hỗ trợ của Apple Home / Google Home / Alexa / SmartThings phải kiểm tra trên đúng phiên bản controller; nhiều controller có thể chưa hiển thị loại này. |
| Khóa điện | Door Lock `0x000A` (`endpoint::door_lock`) | Door Lock | Có example `examples/door_lock` trong esp-matter. |

Không ánh xạ cửa động cơ thành Door Lock chỉ vì cùng có "mở/đóng": Door Lock mô tả
trạng thái chốt khóa (locked/unlocked/jammed), không mô tả hành trình cánh cửa.
Cũng không dùng Window Covering cho cửa đi: controller sẽ coi đó là rèm và có thể
cho phép tự động hóa/ngữ cảnh không phù hợp với một cửa có người đi qua.

## 2. Điểm mở rộng trong mã

```
components/cover_core        state machine thuần C (open/close/stop/limit/timeout/obstacle)
components/cover_hal         motor_driver_t (interface), inputs, pin_guard, sim
components/cover_controller  task + queue + nút bấm, không phụ thuộc Matter
components/cover_storage     NVS
main/app_matter_cover.cpp    ánh xạ Matter cho rèm
main/Kconfig.projbuild       choice APP_DEVICE_PROFILE
```

### Cửa chạy động cơ (door / gate)

- Tái sử dụng: `cover_sm` (hành trình, dead time, công tắc hành trình, timeout,
  vật cản), `motor_driver_t`, `cover_controller`.
- Cần bổ sung / quyết định trước khi viết code:
  - **Chính sách vật cản.** Hiện tại: dừng và chốt lỗi, không đảo chiều. Với cửa,
    tiêu chuẩn áp dụng (ví dụ EN 12453 / UL 325 tùy thị trường) có thể yêu cầu đảo
    chiều một đoạn, giới hạn lực, cảm biến quang hai cấp... Đây là quyết định an
    toàn phải làm cùng thông tin cơ cấu cửa, không được đoán.
  - Đầu vào khóa cơ / tay mở khẩn cấp, cảnh báo đèn/còi trước khi chạy.
  - Ánh xạ trạng thái sang `ClosureControl` (MainState, OverallCurrentState /
    OverallTargetState) và `ClosureDimension` (CurrentState/TargetState) của
    Matter 1.5. Phải đọc header cluster trong `connectedhomeip/src/app/clusters/
    closure-control-server/` của đúng commit SDK (9c2a63634e) trước khi viết.
- Thêm: `main/app_matter_door.cpp`, mục `APP_DEVICE_DOOR` trong
  `choice APP_DEVICE_PROFILE`, bật `CONFIG_SUPPORT_CLOSURE_CONTROL_CLUSTER` và
  `CONFIG_SUPPORT_CLOSURE_DIMENSION_CLUSTER` (đang tắt trong `sdkconfig.defaults`).

### Khóa điện (lock)

- Không dùng `cover_sm`: khóa có trạng thái rời rạc (locked / unlocked / jammed /
  not fully locked), thường là xung điện từ hoặc động cơ nhỏ + công tắc chốt.
- Cần một state machine riêng `lock_sm`: lệnh LOCK/UNLOCK → bật cơ cấu tối đa T ms
  → chờ **cảm biến chốt** xác nhận → báo `LockState` theo cảm biến. Hết thời gian
  mà cảm biến không xác nhận → `LockState = NotFullyLocked`/`Unlatched` hoặc sự kiện
  `LockOperationError` / alarm `LockJammed`. **Không báo "đã khóa" chỉ vì đã gửi lệnh.**
- Matter: `endpoint::door_lock`, delegate/callback của Door Lock cluster
  (tham khảo `examples/door_lock` trong esp-matter release/v1.5). Một số controller
  bắt buộc PIN để mở khóa từ xa; xem cấu hình `RequirePINforRemoteOperation`.
- `motor_driver_t` vẫn dùng được cho khóa dùng động cơ đảo chiều.

## 3. Thông tin cần có trước khi triển khai

Cửa: loại cơ cấu (trượt, cánh mở, cuốn), động cơ và mạch công suất, cảm biến vật
cản (quang, mép an toàn, dòng điện), yêu cầu tiêu chuẩn của thị trường, có encoder
hay không, khóa cơ.

Khóa: loại cơ cấu (solenoid, motor), điện áp/dòng, thời gian kích tối đa, cảm biến
trạng thái chốt và cửa, nguồn dự phòng khi mất điện (fail-safe hay fail-secure).
