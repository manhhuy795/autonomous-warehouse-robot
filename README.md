# Robot Line - WMS Autonomous Robot System

Project mô phỏng và triển khai hệ thống robot line trong kho thông minh. Hệ thống gồm robot bám line dùng ESP32-S3, Arduino UNO đọc cảm biến siêu âm, web dashboard WMS để giám sát/điều phối và AI server xử lý camera, telemetry, tác vụ robot.

## Tổng Quan Hệ Thống

| Thành phần | Thư mục | Vai trò |
| --- | --- | --- |
| ESP32-S3 Robot Controller | `ESP/` | Điều khiển robot bám line, motor, encoder, IR sensor, camera, Wi-Fi, gửi telemetry và nhận lệnh từ server |
| Arduino UNO Ultrasonic Module | `UNO/` | Đọc cảm biến siêu âm HC-SR04 và gửi khoảng cách sang ESP32-S3 qua UART |
| WMS Web Dashboard | `web/` | Giao diện quản lý kho, robot, bản đồ, task, cảnh báo, camera và telemetry |
| AI / API Server | `web/ai-server/` | FastAPI server nhận frame camera, xử lý vision, quản lý robot command, telemetry và dữ liệu WMS |

## Cấu Trúc Thư Mục

```text
robot line/
├── ESP/
│   ├── platformio.ini
│   └── src/main.cpp
├── UNO/
│   ├── platformio.ini
│   └── src/main.cpp
├── web/
│   ├── src/
│   ├── public/
│   ├── ai-server/
│   ├── package.json
│   └── README.md
└── README.md
```

## Công Nghệ Sử Dụng

### Firmware

- PlatformIO
- Arduino framework
- ESP32-S3 DevKitC-1
- Arduino UNO
- ESP32 Camera
- ArduinoJson
- Wi-Fi HTTP client
- IR line sensors
- Encoder
- HC-SR04 ultrasonic sensor

### Web Dashboard

- React
- TypeScript
- Vite
- React Router
- Recharts
- Tailwind CSS
- Lucide React

### AI/API Server

- Python
- FastAPI
- Uvicorn
- OpenCV
- NumPy
- Ultralytics
- WebSocket / HTTP API

## Luồng Hoạt Động

1. UNO đọc khoảng cách từ cảm biến siêu âm HC-SR04.
2. UNO gửi dữ liệu sang ESP32-S3 qua UART với format:

```text
DIST:23.4
DIST:ERROR
```

3. ESP32-S3 điều khiển robot bám line bằng cảm biến IR, motor và encoder.
4. ESP32-S3 gửi telemetry lên AI/API server.
5. ESP32-S3 lấy lệnh điều phối từ server qua API.
6. Camera ESP32-S3 gửi frame về AI server để xử lý hình ảnh.
7. Web dashboard hiển thị trạng thái robot, task, inventory, cảnh báo, camera và bản đồ kho.

## Cài Đặt Firmware ESP32-S3

Yêu cầu:

- VS Code
- PlatformIO extension
- Board ESP32-S3 DevKitC-1

Mở thư mục:

```text
ESP/
```

Kiểm tra file:

```text
ESP/platformio.ini
```

Board hiện tại:

```ini
[env:esp32-s3-n16r8]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
```

Trước khi upload, sửa cấu hình Wi-Fi và IP server trong:

```text
ESP/src/main.cpp
```

Ví dụ:

```cpp
#define WIFI_SSID "YOUR_WIFI"
#define WIFI_PASSWORD "YOUR_PASSWORD"
#define AI_SERVER_BASE_URL "http://YOUR_SERVER_IP:8000"
```

Sau đó build/upload bằng PlatformIO:

```bash
pio run
pio run --target upload
pio device monitor
```

## Cài Đặt Firmware Arduino UNO

Mở thư mục:

```text
UNO/
```

UNO dùng HC-SR04:

```cpp
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;
```

UNO gửi UART sang ESP32-S3:

```cpp
SoftwareSerial espSerial(2, 3); // RX, TX
```

Kết nối khuyến nghị:

```text
UNO D3 TX  -> ESP32-S3 RX qua chia áp 5V xuống 3.3V
UNO GND    -> ESP32-S3 GND
HC-SR04 TRIG -> UNO D9
HC-SR04 ECHO -> UNO D10
```

Build/upload:

```bash
pio run
pio run --target upload
pio device monitor
```

## Chạy Web Dashboard

Vào thư mục:

```bash
cd web
```

Cài dependencies:

```bash
npm install
```

Chạy development server:

```bash
npm run dev
```

Build production:

```bash
npm run build
```

Preview bản build:

```bash
npm run preview
```

## Chạy AI/API Server

Vào thư mục:

```bash
cd web/ai-server
```

Tạo môi trường Python nếu cần:

```bash
python -m venv .venv
```

Kích hoạt môi trường trên Windows PowerShell:

```powershell
.\.venv\Scripts\Activate.ps1
```

Cài thư viện:

```bash
pip install -r requirements.txt
```

Chạy server:

```bash
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

ESP32-S3 cần trỏ `AI_SERVER_BASE_URL` về IP máy đang chạy server, ví dụ:

```cpp
#define AI_SERVER_BASE_URL "http://192.168.1.130:8000"
```

## API Robot Chính

ESP32-S3 đang dùng các endpoint:

```text
GET  /api/robot/command?robotId=robot-01
POST /api/robot/ack
POST /api/robot/telemetry
POST /api/frame
```

Trong đó:

- `/api/frame`: nhận frame camera từ robot.
- `/api/robot/command`: robot hỏi lệnh mới từ server.
- `/api/robot/ack`: robot xác nhận đã nhận/xử lý lệnh.
- `/api/robot/telemetry`: robot gửi trạng thái vận hành.

## Quy Trình Chạy Khuyến Nghị

1. Chạy AI/API server trong `web/ai-server`.
2. Chạy web dashboard trong `web`.
3. Upload firmware UNO để đọc cảm biến siêu âm.
4. Upload firmware ESP32-S3 sau khi chỉnh Wi-Fi và IP server.
5. Mở Serial Monitor để kiểm tra robot kết nối Wi-Fi, camera, telemetry và command polling.
6. Mở dashboard để theo dõi robot, task, camera và cảnh báo.

