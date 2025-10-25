// CarController.h

#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include <WebServer.h>

// Định nghĩa kiểu dữ liệu cho hàm callback xử lý lệnh
// Hàm này sẽ được gọi trong chương trình chính (main) khi có lệnh điều khiển gửi đến.
typedef void (*CommandCallback)(String command);

class CarController {
public:
    // Constructor (Hàm khởi tạo)
    CarController(const char* ssid, const char* password);

    // Hàm thiết lập chế độ AP và khởi động server
    void begin();

    // Hàm phải được gọi trong loop() của chương trình chính
    void handle();

    // Hàm thiết lập callback để chương trình chính nhận được lệnh
    void setCommandHandler(CommandCallback callback);

private:
    WebServer server;
    const char* _ssid;
    const char* _password;
    CommandCallback _commandCallback;

    // Các hàm xử lý request HTTP
    void handleRoot();
    void handleControl();
    void handleNotFound();
};

#endif