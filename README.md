# Hướng dẫn Khởi chạy Robot (micro-ROS và Joystick)

Tài liệu này hướng dẫn các bước để khởi chạy hệ thống điều khiển robot. Quy trình này yêu cầu bạn mở **4 cửa sổ terminal riêng biệt** để chạy song song các tiến trình sau:

1.  **micro-ROS Agent:** Cầu nối giữa robot (ESP32) và ROS 2.
2.  **Joy Node:** Trình điều khiển (driver) đọc dữ liệu từ tay cầm.
3.  **Teleop Node:** Node của bạn (`my_joy_teleop`) để chuyển đổi dữ liệu tay cầm thành lệnh.
4.  **Debug:** Một terminal để kiểm tra kết quả.

## 1. Chuẩn bị

Trước khi bắt đầu, hãy đảm bảo bạn đã:
* Cắm robot (ví dụ: ESP32) vào máy tính qua cổng USB.
* Cắm joystick (tay cầm) vào máy tính.
* Biên dịch (build) thành công workspace của bạn (chứa `my_joy_teleop`).

---

## 2. Các bước thực thi

### Terminal 1: Khởi chạy micro-ROS Agent

Terminal này chịu trách nhiệm kết nối với robot của bạn.

**1. Tìm cổng serial của robot:**
Chạy lệnh sau để tìm đường dẫn chính xác đến phần cứng của bạn:

> ls /dev/serial/by-id/*

Bạn sẽ thấy một kết quả, ví dụ: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

2. Sao chép đường dẫn và khởi chạy agent:
> # THAY THẾ --dev /dev/serial/by-id/... bằng đường dẫn bạn vừa tìm thấy
> ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0

Lưu ý: Hãy để terminal này chạy. Nếu nó kết nối thành công, bạn sẽ thấy thông báo "Session established".

### Terminal 2: Khởi chạy Trình điều khiển Joystick
Terminal này đọc tín hiệu thô từ tay cầm của bạn.

1. Cấp quyền cho joystick: Lệnh này chỉ cần chạy một lần sau khi cắm tay cầm.
> sudo chmod a+r /dev/input/js0

2. Chạy node joy_linux:
> ros2 run joy_linux joy_linux_node

Lưu ý: Hãy để terminal này chạy. Nó đang liên tục phát (publish) dữ liệu tay cầm lên topic /joy.

### Terminal 3: Khởi chạy Node Teleop (Của bạn)
Terminal này chạy code my_joy_teleop để chuyển đổi topic /joy thành topic /cmd_vel.

1. Source workspace của bạn:
> # Đảm bảo bạn ở đúng thư mục workspace
> cd ~/workspace/MangoMobileRobot 
> source install/setup.bash

2. Chạy node của bạn:
> ros2 run my_joy_teleop my_teleop_node

Lưu ý: Hãy để terminal này chạy. Đây là bộ não chuyển đổi lệnh điều khiển.

### Terminal 4: Kiểm tra (Debug)
Terminal này dùng để nghe topic /cmd_vel và xác nhận rằng mọi thứ đang hoạt động.

1. Source workspace của bạn:
> # Đảm bảo bạn ở đúng thư mục workspace
> cd ~/workspace/MangoMobileRobot 
> source install/setup.bash

2. Chạy topic echo:
> ros2 topic echo /cmd_vel

Bây giờ, hãy giữ nút an toàn (deadman button) trên tay cầm và di chuyển cần analog. Nếu mọi thứ thành công, bạn sẽ thấy các giá trị linear.x và angular.z xuất hiện trong terminal này.

