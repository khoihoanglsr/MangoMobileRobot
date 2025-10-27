// CarController.cpp

#include "CarController.h"
#include <WiFi.h>
#include <Arduino.h> // Cần thiết cho Serial.print

// --- CHUỖI HTML GIAO DIỆN WEB (TỪ CODE CỦA BẠN) ---
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Gamepad Car Control - Minimal</title>
    <style>
        /* GHI CHÚ: CSS - Tối ưu cho màn hình ngang (Landscape) */
        body {
            font-family: Arial, sans-serif;
            display: flex;
            justify-content: center;
            align-items: center;
            height: 100vh; 
            margin: 0;
            background-color: #1a1a2e; /* Nền tối */
            color: #ecf0f1;
            overflow: hidden; 
            touch-action: manipulation; /* Tăng phản hồi chạm */
        }

        .container {
            width: 98vw;
            height: 98vh; 
            padding: 10px;
            box-sizing: border-box;
            display: grid;
            /* Chia 3 cột: D-Pad, Trung tâm, Khoảng trống */
            grid-template-columns: 1fr 1fr 1fr; 
            grid-template-rows: 1fr;
            gap: 10px;
        }
        
        /* Khu vực Điều khiển chính (D-Pad và Trống) */
        .gamepad-controls-section {
            display: flex;
            align-items: center;
            justify-content: center;
            background-color: #24243a;
            border-radius: 15px;
        }

        /* Khu vực trung tâm (RPM và Motor Toggle) */
        .center-area {
            display: flex;
            flex-direction: column;
            justify-content: space-around;
            align-items: center;
            padding: 10px;
            gap: 15px;
        }

        /* A. Ô Tốc độ RPM */
        .speed-display {
            text-align: center;
            padding: 10px;
            background-color: #0f3460; 
            border-radius: 12px;
            border: 2px solid #53d0f0;
            width: 90%;
            height: 30%;
        }

        .speed-value {
            font-size: 2.5em; 
            font-weight: bold;
            color: #53d0f0;
        }

        /* D. Motor On/Off - Nút ở giữa */
        #motor-toggle {
            width: 90%;
            height: 40%;
            min-height: 50px;
            font-size: 1.4em;
            font-weight: bold;
            border-radius: 10px;
            transition: background-color 0.3s;
            border: none;
            color: white;
            cursor: pointer;
        }

        #motor-toggle.ON { background-color: #c0392b; } /* Đỏ khi ON */
        #motor-toggle.OFF { background-color: #2ecc71; } /* Xanh lá khi OFF */


        /* B. D-Pad (Điều hướng - Trái) */
        .dpad-controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            grid-template-rows: repeat(3, 1fr);
            gap: 5px;
            justify-items: center;
            align-items: center;
            width: 70%;
            height: 70%;
            max-width: 200px;
            max-height: 200px;
        }
        
        /* Bố trí D-Pad */
        #up-btn    { grid-column: 2; grid-row: 1; }
        #left-btn  { grid-column: 1; grid-row: 2; }
        #right-btn { grid-column: 3; grid-row: 2; }
        #down-btn  { grid-column: 2; grid-row: 3; }
        
        .control-button {
            width: 100%;
            height: 100%;
            font-size: 1.0em;
            cursor: pointer;
            border: none;
            color: white;
            transition: background-color 0.1s, transform 0.1s;
            border-radius: 10px; 
            touch-action: manipulation;
        }

        .dpad-btn {
            background-color: #3f72af;
        }

        .dpad-btn:active, .dpad-btn.active {
            background-color: #112d4e;
            transform: scale(0.90);
        }
        
        /* C. Khoảng trống */
        .right-area-placeholder {
            background-color: #24243a; 
            border-radius: 15px;
        }

    </style>
</head>
<body>

    <div class="container">
        
        <div class="gamepad-controls-section dpad-controls-wrapper">
            <div class="dpad-controls">
                <button class="control-button dpad-btn" id="up-btn" data-cmd="F">↑</button>
                <button class="control-button dpad-btn" id="left-btn" data-cmd="L">←</button>
                <button class="control-button dpad-btn" id="right-btn" data-cmd="R">→</button>
                <button class="control-button dpad-btn" id="down-btn" data-cmd="B">↓</button>
            </div>
        </div>

        <div class="center-area">
            <div class="speed-display">
                <small>Tốc độ hiện tại</small>
                <div class="speed-value"><span id="current-rpm">0</span> RPM</div>
            </div>
            
            <button class="OFF" id="motor-toggle" data-state="0">MOTOR OFF</button>
        </div>

        <div class="right-area-placeholder">
        </div>
    </div>

    <script>
        // GHI CHÚ: JavaScript - Xử lý sự kiện nút và gửi lệnh
        const MOTOR_TOGGLE = document.getElementById('motor-toggle');
        const DIRECTION_BUTTONS = document.querySelectorAll('.dpad-btn');
        const STOP_COMMAND = 'S'; // Lệnh dừng chung

        // Hàm gửi lệnh đến ESP32 qua AJAX (Fetch API)
        function sendCommand(cmd) {
            fetch('/control?cmd=' + cmd)
                .catch(error => {
                    console.error('Lỗi kết nối:', error);
                });
        }
        
        // --- 1. Điều khiển Motor ON/OFF ---
        MOTOR_TOGGLE.addEventListener('click', () => {
            let currentState = MOTOR_TOGGLE.getAttribute('data-state');
            let newState = (currentState === '0') ? '1' : '0';
            let cmd = (newState === '1') ? 'MOTOR_ON' : 'MOTOR_OFF';

            sendCommand(cmd);
            MOTOR_TOGGLE.setAttribute('data-state', newState);
            MOTOR_TOGGLE.textContent = (newState === '1') ? 'MOTOR ON' : 'MOTOR OFF';
            MOTOR_TOGGLE.className = (newState === '1' ? 'ON' : 'OFF');
        });


        // --- 2. Điều khiển Hướng (Nhấn giữ/thả) ---
        DIRECTION_BUTTONS.forEach(button => {
            const cmd = button.getAttribute('data-cmd'); 

            const startHandler = (e) => {
                if(e.type === 'touchstart') e.preventDefault();
                if (MOTOR_TOGGLE.getAttribute('data-state') === '0') return; 
                sendCommand(cmd);
                button.classList.add('active');
            };

            const releaseHandler = (e) => {
                const button = e.currentTarget;
                button.classList.remove('active');
                sendCommand(STOP_COMMAND);
            };

            button.addEventListener('mousedown', startHandler);
            button.addEventListener('touchstart', startHandler, {passive: false});

            button.addEventListener('mouseup', releaseHandler);
            button.addEventListener('touchend', releaseHandler);
            button.addEventListener('mouseleave', (e) => {
                if(e.currentTarget.classList.contains('dpad-btn')) {
                   releaseHandler(e);
                }
            });
        });
        
    </script>
</body>
</html>
)rawliteral";
// --- HẾT CHUỖI HTML GIAO DIỆN WEB ---


// Hàm xử lý HTTP Request (Cần bind vào object của lớp)
void CarController::handleRoot() {
    server.send(200, "text/html", MAIN_page);
}

void CarController::handleControl() {
    if (server.hasArg("cmd")) {
        String cmd = server.arg("cmd");
        
        // GỌI HÀM CALLBACK: Gửi lệnh về chương trình chính (main)
        if (_commandCallback) {
            _commandCallback(cmd);
        }

        server.send(200, "text/plain", "OK");
        return;
    }
    server.send(400, "text/plain", "Bad Request");
}

void CarController::handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

// Khởi tạo Class
CarController::CarController(const char* ssid, const char* password) : server(80) {
    _ssid = ssid;
    _password = password;
    _commandCallback = nullptr; // Khởi tạo callback là NULL
}

// Thiết lập callback
void CarController::setCommandHandler(CommandCallback callback) {
    _commandCallback = callback;
}

void CarController::begin() {
    Serial.begin(115200);
    
    // Thiết lập Access Point (AP)
    Serial.print("Setting up AP: ");
    Serial.println(_ssid);
    WiFi.softAP(_ssid, _password);

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    // Thiết lập đường dẫn (Routes) và bind vào hàm xử lý của Class
    // Cần sử dụng std::bind để hàm của Class có thể hoạt động với server.on()
    server.on("/", std::bind(&CarController::handleRoot, this));
    server.on("/control", std::bind(&CarController::handleControl, this));
    server.onNotFound(std::bind(&CarController::handleNotFound, this));

    server.begin();
    Serial.println("HTTP server started");
}

void CarController::handle() {
    server.handleClient();
}