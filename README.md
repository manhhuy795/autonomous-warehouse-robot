# Robot Line - WMS Autonomous Robot System

Project mô phỏng và triển khai hệ thống robot line trong kho thông minh. Hệ thống gồm robot bám line sử dụng ESP32-S3, Arduino UNO đọc cảm biến siêu âm, cánh tay robot để gắp và thả hàng hóa, web dashboard WMS để giám sát/điều phối và AI server xử lý camera, telemetry, tác vụ robot.

Robot có khả năng di chuyển theo line, xử lý line đứt đoạn, né tránh vật cản và tự tìm lại line sau khi hoàn thành quá trình tránh vật cản. AI server hỗ trợ xử lý hình ảnh từ camera để nhận diện vật cản, người và hàng hóa trong môi trường kho.

## Tổng Quan Hệ Thống

| Thành phần                    | Thư mục          | Vai trò                                                                                                   |
| ----------------------------- | ---------------- | --------------------------------------------------------------------------------------------------------- |
| ESP32-S3 Robot Controller     | `ESP/`           | Điều khiển robot bám line, motor, encoder, IR sensor, camera, Wi-Fi, gửi telemetry và nhận lệnh từ server |
| Arduino UNO Ultrasonic Module | `UNO/`           | Đọc cảm biến siêu âm HC-SR04 và gửi khoảng cách sang ESP32-S3 qua UART                                    |
| Robot Arm Module              | `ESP/`           | Điều khiển cánh tay robot để gắp và thả hàng hóa trong kho                                                |
| WMS Web Dashboard             | `web/`           | Giao diện quản lý kho, robot, bản đồ, task, cảnh báo, camera và telemetry                                 |
| AI / API Server               | `web/ai-server/` | FastAPI server nhận frame camera, xử lý vision, quản lý robot command, telemetry và dữ liệu WMS           |

## Tính Năng Chính

* Robot bám line tự động trong môi trường kho.
* Hỗ trợ di chuyển qua các đoạn line bị đứt hoặc không liên tục.
* Phát hiện và né tránh vật cản trong quá trình di chuyển.
* Tự tìm lại line sau khi hoàn thành quá trình tránh vật cản.
* Cánh tay robot hỗ trợ gắp và thả hàng hóa tại các vị trí được chỉ định.
* AI server xử lý hình ảnh từ camera để nhận diện vật cản, người và hàng hóa.
* Web dashboard WMS hỗ trợ giám sát robot, task, inventory, cảnh báo, camera và telemetry.
* ESP32-S3 gửi telemetry và nhận lệnh điều phối từ server thông qua API.
