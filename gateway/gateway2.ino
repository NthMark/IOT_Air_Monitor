#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "BluetoothSerial.h"

// --- CẤU HÌNH WIFI & FIREBASE ---
#define WIFI_SSID "UET-Wifi-Office-Free 2.4Ghz" 
#define WIFI_PASSWORD ""
#define FIREBASE_URL "https://iotsensor-943bb-default-rtdb.firebaseio.com/" 
#define FIREBASE_SECRET "GMNDyWiMcNNCgX0vXkd5RlxNeDDMvwVhVIN0eGb7"

// --- CẤU HÌNH BLUETOOTH ---
static const char *HC05_NAME = "uwb_log";
static const char *HC05_PIN  = "1234";

// --- BIẾN TOÀN CỤC ---
BluetoothSerial SerialBT;
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

float current_pm25 = -1.0;
float current_co = -1.0;

unsigned long lastFirebaseSend = 0;
const unsigned long FIREBASE_INTERVAL = 1000; // Tần suất 1 giây (1000ms)

// Bộ đệm Bluetooth
char s_line_buf[192];
size_t s_line_len = 0;

// --- HÀM PHÂN TÍCH DỮ LIỆU ---
void handle_bluetooth_data(const char *line) {
    float val = 0;
    // Tìm và lấy giá trị CO (MQ7)
    if (strstr(line, "MQ7_SS:")) {
        char *p = strstr(line, "ppm=");
        if (p && sscanf(p, "ppm=%f", &val) == 1) {
            current_co = val;
            Serial.printf("[BT] Nhận CO: %.2f ppm\n", current_co);
        }
    }
    // Tìm và lấy giá trị PM2.5 (GP2Y)
    else if (strstr(line, "GP2Y_SS:")) {
        char *p = strstr(line, "dust=");
        if (p && sscanf(p, "dust=%f", &val) == 1) {
            current_pm25 = val;
            Serial.printf("[BT] Nhận PM2.5: %.3f mg/m3\n", current_pm25);
        }
    }
}

// --- HÀM GỬI FIREBASE (CHẠY SONG SONG TRONG LOOP) ---
void updateFirebaseTask() {
    if (millis() - lastFirebaseSend >= FIREBASE_INTERVAL) {
        lastFirebaseSend = millis();

        // Chỉ gửi khi đã có dữ liệu từ Bluetooth
        if (current_pm25 != -1.0 && current_co != -1.0) {
            Serial.println(">>> Đang cập nhật Firebase...");
            
            // Gửi PM2.5
            Firebase.setFloat(firebaseData, "/SensorData/PM25", current_pm25);
            // Gửi CO
            Firebase.setFloat(firebaseData, "/SensorData/CO", current_co);
            
            Serial.println(">>> Đã cập nhật xong (1s).");
        } else {
            Serial.println("--- Đang đợi dữ liệu đủ 2 cảm biến ---");
        }
    }
}

// --- KẾT NỐI BLUETOOTH ---
bool checkBluetooth() {
    if (SerialBT.connected()) return true;
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {
        lastAttempt = millis();
        Serial.println("BT: Đang tìm kết nối với HC-05...");
        return SerialBT.connect(HC05_NAME);
    }
    return false;
}

void setup() {
    Serial.begin(115200);

    // WiFi & Firebase Setup
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi OK!");

    config.database_url = FIREBASE_URL;
    config.signer.tokens.legacy_token = FIREBASE_SECRET;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    // Bluetooth Setup
    SerialBT.begin("ESP32_GATEWAY", true);
    SerialBT.setPin(HC05_PIN, strlen(HC05_PIN));
    checkBluetooth();
}

void loop() {
    // 1. Luôn ưu tiên đọc dữ liệu Bluetooth
    if (checkBluetooth()) {
        while (SerialBT.available() > 0) {
            char c = (char)SerialBT.read();
            if (c == '\n') {
                s_line_buf[s_line_len] = '\0';
                handle_bluetooth_data(s_line_buf);
                s_line_len = 0;
            } else if (c != '\r' && s_line_len < 191) {
                s_line_buf[s_line_len++] = c;
            }
        }
    }

    // 2. Chạy hàm cập nhật Firebase (Tần suất 1s/lần)
    updateFirebaseTask();
}
