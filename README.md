# auto-door — firmware ESP32-C6 điều khiển rèm qua Matter over Wi-Fi

Firmware ESP-IDF + ESP-Matter (không Arduino) cho ESP32-C6. Profile đã triển khai:
**rèm cuốn / rèm kéo** (Matter *Window Covering*). Cửa chạy động cơ và khóa điện mới
có thiết kế điểm mở rộng — xem [docs/EXTENDING.md](docs/EXTENDING.md).

> **Mặc định chạy ở CHẾ ĐỘ MÔ PHỎNG**: không cấu hình và không điều khiển bất kỳ
> GPIO động cơ nào. Chỉ chuyển sang phần cứng sau khi có sơ đồ mạch (mục 4).
>
> Firmware này **chưa được chứng nhận Matter** và chưa qua kiểm thử trên phần cứng
> thật. Xem mục 9 "Đã kiểm chứng gì".

---

## 1. Phiên bản SDK đã kiểm chứng

| Thành phần | Phiên bản | Commit |
|---|---|---|
| ESP-Matter | nhánh `release/v1.5` (ESP-Matter không gắn tag git; component registry tương ứng là `espressif/esp_matter` 1.5.x) | `7ab5228aa238e837e948e840c380e8beb601db9a` (2026-10-19) |
| connectedhomeip (submodule của ESP-Matter) | theo ESP-Matter | `9c2a63634e880b92e29b25a49543b70a4baf1572` |
| ESP-IDF | tag `v5.5.5` | `b774170ff46c393eeb5e495ea37936038d3f4f4f` |

Cách chọn: README của nhánh `release/v1.5` ghi *"it is recommended to utilize
ESP-IDF v5.5.5"* và `docs/en/developing.rst` hướng dẫn `git checkout v5.5.5`.
Không dùng cặp mới nhất (`release/v1.6.1` + ESP-IDF v6.0.x) vì ESP-IDF 6.0 có
thay đổi lớn về driver và chưa cần cho dự án này; có thể nâng cấp sau khi kiểm thử lại.

Các API Matter dùng trong mã đã được đối chiếu với mã nguồn của đúng các commit trên:

- `esp_matter::endpoint::window_covering::create()` / `config_t`
  (`components/esp_matter/data_model/esp_matter_endpoint.h`)
- `cluster::window_covering::feature::lift`, `position_aware_lift`,
  `cluster::window_covering::attribute::create_safety_status()`
- `chip::app::Clusters::WindowCovering::Delegate` (`HandleMovement`, `HandleStopMotion`),
  `OperationalStateSet()`, `WC_PERCENT100THS_MIN_OPEN = 0`, `WC_PERCENT100THS_MAX_CLOSED = 10000`
  (`connectedhomeip/src/app/clusters/window-covering-server/`)
- `esp_matter::attribute::set_deferred_persistence()`, `esp_matter::factory_reset()`,
  `esp_matter::console::add_commands()`
- `CommissioningWindowManager::OpenBasicCommissioningWindow(Seconds32, CommissioningWindowAdvertisement)`
- `chip::DeviceLayer::PlatformMgr().ScheduleWork()`

## 2. Kiến trúc

```
                ┌──────────────────────── Matter thread (CHIP task) ────────────────────────┐
 Controller ──► │ window-covering-server (upstream) ──► CoverDelegate::HandleMovement/Stop │
 (Wi-Fi)        │        ▲ attribute writes (sync_work)            │ post (non-blocking)    │
                └────────┼─────────────────────────────────────────┼────────────────────────┘
                         │ PlatformMgr().ScheduleWork()             ▼  queue (STOP → front)
                ┌────────┴────────────── cover_ctl task (prio 10, chu kỳ 10 ms) ─────────────┐
 Nút, công tắc ─► inputs (poll) → debounce → cover_sm (state machine) → motor_driver ─► Mạch công suất
 hành trình,    │                                   │ events                                  │
 vật cản, lỗi   │                                   └──► cover_storage (task NVS riêng)       │
                └────────────────────────────────────────────────────────────────────────────┘
```

| Thư mục | Vai trò |
|---|---|
| `main/app_main.cpp` | Khởi tạo theo thứ tự an toàn: NVS → controller (tắt đầu ra trước) → Matter node → `esp_matter::start()` |
| `main/app_matter_cover.cpp` | Endpoint Window Covering, delegate lệnh, đồng bộ attribute, mở commissioning window |
| `main/app_config.cpp` | Kconfig → cấu hình, kiểm tra GPIO |
| `main/app_console.cpp` | Lệnh `matter esp cover ...` (mô phỏng, hiệu chuẩn, chẩn đoán) |
| `main/Kconfig.projbuild` | Toàn bộ GPIO, mức tác động, thời gian |
| `components/cover_core` | State machine thuần C + debounce, **build và test được trên PC** |
| `components/cover_hal` | Driver động cơ (`gpio2`, `sim`), interlock thứ hai, đọc đầu vào, chặn GPIO cấm, plant mô phỏng |
| `components/cover_controller` | Task điều khiển, queue lệnh, nút bấm, nút chức năng |
| `components/cover_storage` | Hiệu chuẩn + vị trí dừng cuối trong NVS (task ghi riêng) |
| `test/host` | Unit test state machine (gcc, ASan/UBSan) |

### State machine (`cover_sm`)

| Trạng thái | Động cơ | Ý nghĩa |
|---|---|---|
| `UNCALIBRATED` | tắt | dừng, chưa có thời gian hành trình (chỉ mở/đóng hết được) |
| `STOPPED` | tắt | dừng, đã hiệu chuẩn |
| `OPENING` / `CLOSING` | một chiều | đang chạy |
| `DEADTIME` | tắt | đã tắt đầu ra, đang chờ khoảng nghỉ trước khi chạy lệnh chờ |
| `FAULT` | tắt | lỗi đã chốt; chỉ lệnh STOP xóa được, và chỉ khi nguyên nhân đã hết |

Quy tắc bảo vệ (được kiểm thử trong `test/host`):

- Đầu ra là một giá trị `cover_dir_t` duy nhất → không thể yêu cầu hai chiều cùng lúc.
  Driver có **interlock độc lập thứ hai** (`motor_interlock_*`): từ chối bật chiều mới khi
  chiều kia đang bật hoặc chưa đủ dead time, khi đó ép cả hai đầu ra về tắt và báo
  `FAULT(OUTPUT)`. Driver `gpio2` còn đọc lại mức chân sau mỗi lần ghi.
- Mọi lần khởi động động cơ (kể cả đảo chiều, kể cả sau khi boot) đều cách lần tắt
  trước ít nhất `COVER_DEADTIME_MS`.
- Boot/reset: đầu ra được ghi mức không tác động **trước khi** bật chế độ output; không
  bao giờ tự chạy lại; nếu reset/mất điện khi đang chạy thì vị trí = *không xác định*.
- Dừng khi: chạm công tắc hành trình, quá thời gian, vật cản, đầu vào lỗi driver, lỗi
  đầu ra, giữ nút STOP. Hai công tắc hành trình cùng tác động → `FAULT(LIMIT_BOTH)`;
  chạm công tắc ngược chiều → `FAULT(LIMIT_WRONG)` (thường do đấu ngược động cơ).
- Vật cản: **chỉ dừng và chốt lỗi, không tự đảo chiều.** Khi vật cản còn tác động thì
  mọi lệnh chạy bị từ chối.
- STOP được đưa vào **đầu** queue; nút STOP được xử lý trong vòng điều khiển; nút STOP
  đang giữ chặn mọi lệnh chạy. Nhấn nút OPEN/CLOSE khi đang chạy = dừng.
- Callback Matter không có vòng lặp/delay: chỉ đọc target và `xQueueSend(..., 0)`.

## 3. Ánh xạ Matter

- Device type: **Window Covering (0x0202)**, endpoint 1, feature `Lift` + `PositionAwareLift`.
- Lệnh: `UpOrOpen`, `DownOrClose`, `StopMotion`, `GoToLiftPercentage`.
- **Quy ước phần trăm (đã kiểm tra trong SDK):** `0` = mở hoàn toàn, `10000` (100.00 %) =
  đóng hoàn toàn (`WC_PERCENT100THS_MIN_OPEN = 0`, `WC_PERCENT100THS_MAX_CLOSED = 10000`).
  Giá trị là phần trăm **đóng**. Trong firmware, vị trí nội bộ dùng đúng quy ước này.
  Ứng dụng controller có thể hiển thị ngược lại ("mở 100 %"), đó là việc của controller.
- `TargetPositionLiftPercent100ths`: đích mà động cơ **đang** thực hiện (do controller hay
  nút yêu cầu). Khi dừng: bằng vị trí hiện tại (giống `StopMotion` của SDK), hoặc `null`
  nếu chưa biết vị trí. Lệnh bị từ chối (đang lỗi, chưa hiệu chuẩn...) → target được đặt
  lại bằng vị trí hiện tại để controller thấy lệnh không được thực hiện.
- `CurrentPositionLiftPercent100ths`: vị trí của state machine; `null` khi không xác định.
- `OperationalStatus`: theo trạng thái động cơ thật (kể cả `DEADTIME` = đang chuẩn bị chạy).
- `SafetyStatus`: `ObstacleDetected`, `StopInput`, `MotorJammed`+`PositionFailure` (timeout),
  `PositionFailure`+`HardwareFailure` (công tắc hành trình bất hợp lý), `HardwareFailure`.
- `ConfigStatus`: `Operational` + `LiftPositionAware`; **`LiftEncoderControlled` để 0** —
  đúng với vị trí ước lượng theo thời gian.
- Không có vòng lặp: thay đổi attribute không gọi delegate; callback attribute của ứng
  dụng bỏ qua cluster này; mọi ghi attribute chạy trên Matter thread qua `ScheduleWork`
  (đã giữ stack lock), task điều khiển không bao giờ chờ khóa Matter.
- Lưu NVS: `CurrentPositionLiftPercent100ths` là non-volatile trong esp-matter nên được
  đặt *deferred persistence*; sau `esp_matter::start()` firmware ghi đè giá trị cũ bằng
  trạng thái thật.

Giới hạn controller: hầu hết hệ sinh thái (Apple Home, Google Home, Alexa, SmartThings,
Home Assistant) hỗ trợ Window Covering lift; giao diện và việc có nút "Stop" hay không
tùy controller. Nếu controller không có nút Stop thì lỗi chốt chỉ xóa được bằng nút STOP
vật lý hoặc console. Hãy kiểm tra trên đúng phiên bản controller bạn dùng.

## 4. Phần cứng

### 4.1 Đấu nối logic (chưa có sơ đồ mạch — mọi GPIO mặc định `-1`)

| Tín hiệu | Kconfig | Hướng | Mặc định | Ghi chú |
|---|---|---|---|---|
| Chiều MỞ | `COVER_GPIO_OUT_OPEN` | ra | -1 | chỉ dùng khi tắt mô phỏng |
| Chiều ĐÓNG | `COVER_GPIO_OUT_CLOSE` | ra | -1 | |
| Mức tác động đầu ra | `COVER_OUT_ACTIVE_HIGH` | | y | |
| Nút MỞ / ĐÓNG / DỪNG | `COVER_GPIO_BTN_OPEN/CLOSE/STOP` | vào | -1 | mặc định nút nối GND, pull-up trong |
| Nút chức năng | `COVER_GPIO_BTN_FUNC` | vào | -1 | commissioning / factory reset |
| Công tắc hành trình mở / đóng | `COVER_GPIO_LIMIT_OPEN/CLOSED` | vào | -1 | khuyên dùng tiếp điểm **NC** → `ACTIVE_HIGH=y` (đứt dây = dừng) |
| Vật cản | `COVER_GPIO_OBSTACLE` | vào | -1 | |
| Lỗi mạch công suất | `COVER_GPIO_DRIVER_FAULT` | vào | -1 | nhiều H-bridge có nFAULT tích cực mức thấp |

Firmware **từ chối** khi chạy (log lỗi, không điều khiển động cơ) nếu chọn chân xung đột
của ESP32-C6 (theo bảng GPIO của ESP-IDF v5.5): strapping GPIO4, 5, 8, 9, 15 (chỉ cho
phép làm *đầu vào* khi bật `COVER_ALLOW_STRAPPING_INPUTS`), SPI flash GPIO24–30,
USB-JTAG GPIO12–13, UART0 console GPIO16–17, chân trùng nhau. Các linh kiện riêng của
board (LED RGB trên GPIO8 của DevKitC-1, nút BOOT trên GPIO9...) phải tự đối chiếu với
sơ đồ board.

### 4.2 Mạch công suất mà driver `gpio2` yêu cầu

Driver `gpio2` chỉ phù hợp với mạch có **hai đầu vào logic độc lập, mỗi đầu vào bật một
chiều, cả hai tắt = động cơ dừng an toàn**:

- động cơ ống AC 2 chiều (dây chung + dây mở + dây đóng) qua 2 relay/SSR — **nên có
  interlock phần cứng** (relay thứ hai cấp qua tiếp điểm NC của relay thứ nhất), vì cấp
  đồng thời hai cuộn có thể hỏng động cơ/tụ;
- động cơ DC qua H-bridge có chân IN1/IN2 kiểu "IN1=1 → quay thuận, IN2=1 → quay ngược,
  cả hai 0 → coast/brake" và tự chống shoot-through.

Bắt buộc với mọi mạch: **điện trở kéo ngoài giữ cả hai đầu vào ở mức không tác động**
khi ESP32-C6 đang reset/bootloader (lúc đó GPIO không được điều khiển). Pull nội chỉ là
lớp bảo vệ phụ.

Không phù hợp nếu không viết driver mới: cấu hình "relay chạy + relay đảo chiều"
(run + direction), điều tốc PWM/soft-start, BLDC, động cơ có bộ điều khiển riêng nhận
lệnh dạng xung/RS485/RF, công tắc tường kiểu bập bênh giữ trạng thái (latching).

### 4.3 Thông tin còn thiếu để chuyển sang phần cứng

1. Board/module cụ thể (DevKitC-1 hay mạch riêng), dung lượng flash, GPIO nào còn trống.
2. Loại động cơ (AC ống 230 V có công tắc hành trình trong? DC 12/24 V?), dòng định mức.
3. Mạch công suất: module relay (mức tác động, có interlock?), IC H-bridge (mã, chân
   fault, brake/coast), hay SSR/triac.
4. Có công tắc hành trình ngoài không; NO hay NC; mức điện áp.
5. Cảm biến vật cản (quang, mép an toàn, đo dòng) và kiểu ngõ ra.
6. Nút bấm: số lượng, nhấn nhả hay công tắc giữ trạng thái.
7. Dead time tối thiểu theo datasheet động cơ/relay; thời gian hành trình xấp xỉ.

## 5. Vị trí, hiệu chuẩn, lưu trữ

- **Không có encoder → vị trí giữa hành trình chỉ là ƯỚC LƯỢNG theo thời gian.** Chỉ vị
  trí tại công tắc hành trình là đo được (log ghi `limit switch` / `estimated`).
  Sai số tích lũy theo số lần dừng giữa chừng, tải, điện áp, nhiệt độ; mỗi lần chạy hết
  về một đầu có công tắc thì vị trí được đồng bộ lại.
- **Hiệu chuẩn tự động** (cần cả hai công tắc hành trình): giữ đồng thời MỞ+ĐÓNG
  `COVER_CAL_COMBO_HOLD_MS` (5 s) hoặc `matter esp cover cal start`. Trình tự: chạy về
  công tắc MỞ → đo thời gian MỞ→ĐÓNG → đo ĐÓNG→MỞ → lưu NVS. Bất kỳ lệnh/nút nào trong
  lúc hiệu chuẩn sẽ hủy hiệu chuẩn và dừng.
- **Không có công tắc hành trình**: nhập thời gian đo bằng đồng hồ:
  `matter esp cover cal set <open_ms> <close_ms>` hoặc `COVER_DEFAULT_OPEN_MS/CLOSE_MS`.
  Khi đó đầu hành trình đạt được bằng thời gian + `COVER_END_OVERRUN_PCT`; **chỉ chấp nhận
  với động cơ có công tắc hành trình bên trong.**
- Chưa hiệu chuẩn: chỉ mở/đóng hết (dừng bởi công tắc hoặc `COVER_MAX_RUN_UNCAL_MS`);
  `GoToLiftPercentage` giữa chừng bị từ chối.
- Sau mất điện / reset: nếu bản ghi NVS cho thấy đang chạy → vị trí *không xác định*
  (`null` trên Matter); nếu dừng sạch → khôi phục vị trí *ước lượng*
  (`COVER_RESTORE_POSITION`). **Không bao giờ tự chạy để tìm hành trình**; người dùng ra
  lệnh mở/đóng hết để đồng bộ lại.
- Ghi NVS: hiệu chuẩn chỉ khi thay đổi; vị trí tối đa 2 lần mỗi lần chạy (đánh dấu
  "đang chạy" khi bắt đầu, vị trí khi dừng; bỏ qua nếu trùng), trong task riêng nên vòng
  điều khiển không bị chặn bởi xóa flash. Namespace `cover` không bị factory reset Matter
  xóa, trừ khi bật `COVER_FACTORY_RESET_CLEARS_CALIBRATION`.

## 6. Nút chức năng, commissioning, factory reset

| Thao tác | Kết quả |
|---|---|
| Giữ nút chức năng ≥ 3 s rồi nhả (< 10 s) | Mở commissioning window `COVER_COMMISSIONING_WINDOW_S` (300 s). Chưa ghép: BLE + DNS-SD; đã ghép: DNS-SD (on-network) |
| Giữ ≥ 10 s (log "release now to FACTORY RESET") rồi nhả | Dừng & khóa động cơ → factory reset Matter (xóa fabric, Wi-Fi) → khởi động lại |
| Nhấn ngắn | Không làm gì (log) |
| Giữ MỞ + ĐÓNG 5 s | Hiệu chuẩn |

Hai ngưỡng thời gian khác nhau và firmware từ chối cấu hình có thời gian reset ≤ thời
gian commissioning. Console: `matter esp cover commission`, `matter esp cover factory-reset`.

### Dữ liệu ghép nối: phát triển vs sản xuất

Bản build mặc định dùng **dữ liệu test của SDK** — chỉ để phát triển:

- DAC provider `CONFIG_EXAMPLE_DAC_PROVIDER` (chứng chỉ attestation test),
- VID/PID test `0xFFF1` / `0x8000`,
- passcode `20202021`, discriminator `3840` (0xF00).
  QR code và manual pairing code được in trong log khi khởi động (`SetupQRCode`, `Manual pairing code`).

Controller thương mại có thể cảnh báo "thiết bị chưa được chứng nhận" với dữ liệu này;
dùng chế độ developer / chip-tool để thử.

Khi sản xuất cần thay: VID cấp bởi CSA và PID riêng; DAC/PAI riêng cho từng thiết bị và
Certification Declaration (sinh bằng `esp-matter-mfg-tool`, nạp vào phân vùng
`esp_secure_cert`/`fctry`, chọn `CONFIG_SEC_CERT_DAC_PROVIDER` hoặc
`CONFIG_FACTORY_PARTITION_DAC_PROVIDER` + `CONFIG_ENABLE_ESP32_FACTORY_DATA_PROVIDER`);
passcode/discriminator **ngẫu nhiên, duy nhất** cho từng thiết bị (không dùng 20202021);
bật Secure Boot v2 và Flash Encryption; tắt `CONFIG_ENABLE_CHIP_SHELL`; cấu hình OTA
provider. Thiết bị chỉ được gọi là "Matter certified" sau khi qua phòng thử nghiệm được
CSA ủy quyền và được cấp chứng nhận.

## 7. Cài đặt, build, nạp, xem log

```bash
# 1) ESP-IDF v5.5.5
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32c6 && cd ..

# 2) ESP-Matter release/v1.5 (shallow, chỉ platform esp32 + linux)
source esp-idf/export.sh
git clone -b release/v1.5 --depth 1 https://github.com/espressif/esp-matter.git
cd esp-matter
git checkout 7ab5228aa238e837e948e840c380e8beb601db9a   # nếu muốn cố định đúng commit đã kiểm chứng (cần bỏ --depth 1)
git submodule update --init --depth 1
cd connectedhomeip/connectedhomeip && ./scripts/checkout_submodules.py --platform esp32 linux --shallow && cd ../..
./install.sh            # thêm --no-host-tool nếu không cần chip-tool
cd ..

# 3) Mỗi terminal mới
source esp-idf/export.sh
source esp-matter/export.sh
export IDF_CCACHE_ENABLE=1   # nếu đã cài ccache

# 4) Build / nạp / log
cd auto-door
idf.py set-target esp32c6
idf.py menuconfig             # "Auto-door: ..." menus (tùy chọn)
idf.py build
idf.py -p /dev/ttyUSB0 erase-flash   # lần đầu, xóa NVS cũ
idf.py -p /dev/ttyUSB0 flash monitor # thoát monitor: Ctrl+]
```

Lệnh console (khi `CONFIG_ENABLE_CHIP_SHELL=y`, gõ trong `idf.py monitor`):

```
matter esp cover help
matter esp cover status
matter esp cover open | close | stop
matter esp cover goto 40            # 40 % ĐÓNG (quy ước Matter)
matter esp cover btn open           # mô phỏng nhấn nút vật lý
matter esp cover cal start | cal set 15000 16000 | cal clear
matter esp cover sim obstacle 1     # chỉ ở chế độ mô phỏng
matter esp cover sim limits both | auto
matter esp cover sim jam 1
matter esp cover sim fault 1
```

## 8. Kiểm thử

### 8.1 Unit test state machine trên PC (đã chạy)

```bash
cmake -S test/host -B build-host && cmake --build build-host && ./build-host/test_cover_sm
```

23 nhóm test, ~160 kiểm tra, chạy với AddressSanitizer + UBSan. Mỗi tick đều kiểm tra
bất biến *không bao giờ đảo chiều trực tiếp* và *luôn đủ dead time*. Bao gồm: boot an
toàn, dead time sau boot, lệnh ngược chiều liên tục, dừng khẩn/STOP trong dead time/giữ
nút STOP, công tắc hành trình, timeout (kẹt, chưa hiệu chuẩn), vật cản (không đảo chiều,
chốt lỗi, xóa lỗi), hai công tắc cùng tác động (khi chạy và khi boot), đấu ngược động cơ,
lỗi driver, hiệu chuẩn (đo đúng ±2 tick, hủy giữa chừng, thiếu công tắc), go-to giữa
chừng và đổi đích cùng chiều, reset khi đang chạy, không công tắc hành trình, nút bấm,
dịch chuyển bằng tay, tràn bộ đếm thời gian 32 bit, debounce.

### 8.2 Kiểm thử trên thiết bị ở chế độ mô phỏng (cần làm thủ công)

| Kịch bản | Thao tác | Kỳ vọng |
|---|---|---|
| Lệnh ngược chiều | `open`, sau 2 s `close` | log `[SIM] motor OFF`, ≥ 500 ms sau mới `[SIM] motor CLOSE` |
| Dừng khẩn | `close` rồi `stop` | OFF ngay lập tức |
| Hành trình | `open` | dừng tại `limit switch`, vị trí 0 % |
| Timeout | `sim jam 1`, `close` | `FAULT: run timeout`, Matter SafetyStatus MotorJammed |
| Vật cản | `close`, `sim obstacle 1` | dừng, `FAULT: obstacle`, không đảo chiều; `stop` bị từ chối đến khi `sim obstacle 0` |
| Lỗi cảm biến | `sim limits both` | `FAULT: both limit switches active` |
| Reset khi đang chạy | `close`, nhấn RESET board | sau boot: `position unknown, NOT moving automatically`, động cơ không chạy |

### 8.3 Commissioning và điều khiển Matter (chip-tool)

```bash
chip-tool pairing ble-wifi 0x11 "<SSID>" "<PASSWORD>" 20202021 3840
chip-tool windowcovering down-or-close 0x11 1
chip-tool windowcovering stop-motion 0x11 1
chip-tool windowcovering up-or-open 0x11 1
chip-tool windowcovering go-to-lift-percentage 5000 0x11 1      # cần đã hiệu chuẩn
chip-tool windowcovering read current-position-lift-percent100ths 0x11 1
chip-tool windowcovering read target-position-lift-percent100ths 0x11 1
chip-tool windowcovering read operational-status 0x11 1
chip-tool windowcovering read safety-status 0x11 1
chip-tool windowcovering subscribe current-position-lift-percent100ths 1 10 0x11 1
chip-tool pairing open-commissioning-window 0x11 1 300 1000 3840 # multi-admin
```

Kiểm tra thêm: nút vật lý → attribute trên controller thay đổi (subscribe ở trên);
lệnh khi đang `FAULT` bị từ chối và target quay về vị trí hiện tại.

### 8.4 Mất Wi-Fi và kết nối lại

1. Tắt access point khi rèm đang chạy → rèm vẫn chạy/dừng đúng (công tắc, timeout).
2. Khi mất mạng: **nút vật lý MỞ/ĐÓNG/DỪNG và hiệu chuẩn vẫn phải hoạt động** — toàn bộ
   logic nằm trong task `cover_ctl`, không phụ thuộc Wi-Fi/Matter.
3. Bật lại AP → log `Wi-Fi connectivity: connected`; controller subscribe lại và nhận
   vị trí hiện tại (thay đổi trong lúc mất mạng được ghi khi Matter thread chạy lại).

## 9. Đã kiểm chứng gì / chưa

- **Đã làm (2026-10-22, môi trường Linux không có board):**
  - `idf.py build` cho `esp32c6` với ESP-IDF v5.5.5 + ESP-Matter `release/v1.5@7ab5228a`:
    thành công, không có warning trong mã dự án. Ảnh `auto_door.bin` = 0x1B3690 byte,
    phân vùng app 0x1E0000 → còn trống ~9 %.
  - Build thêm biến thể phần cứng (`COVER_SIMULATION=n`, GPIO giả định chỉ để biên dịch):
    thành công.
  - Unit test state machine trên PC: 23/23 nhóm, 160/160 kiểm tra đạt (ASan + UBSan).
  - `dependencies.lock` được commit để cố định phiên bản các managed component.
- **Chưa làm (không có phần cứng):** chạy trên ESP32-C6 thật, commissioning BLE/Wi-Fi,
  điều khiển từ controller thương mại, đo thời gian thực, kiểm thử với động cơ thật.
  Các mục 8.2–8.4 là hướng dẫn kiểm thử, chưa phải kết quả.
- Không có chứng nhận Matter, không có đánh giá an toàn điện/cơ theo tiêu chuẩn nào.
