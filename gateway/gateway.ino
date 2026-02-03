/*
 * ESP32 Gateway: parse STM32 UART logs and forward only valid sensor data.
 *
 * INPUT (new): Bluetooth SPP via HC-05 (HC-05 is the SPP slave).
 * OUTPUT: Serial (USB).
 *
 * Notes:
 * - Ensure STM32 UART baud matches the HC-05 UART baud (many HC-05 modules default to 9600 in data mode).
 */

#include <Arduino.h>
#include "BluetoothSerial.h"
#include <string.h>

// ---- HC-05 config ----
static const char *HC05_NAME = "uwb_log"; // change if you renamed it
// If name connect doesn't work, set this to your HC-05 MAC (e.g. "98:D3:11:FC:2A:7B") and set HC05_USE_MAC=1.
static const char *HC05_MAC  = "";
static const bool  HC05_USE_MAC = false;
// HC-05 legacy PIN (often "1234" or "0000"). Set to "" if your HC-05 does not require pairing.
static const char *HC05_PIN  = "1234";
static BluetoothSerial SerialBT;

static const size_t LINE_BUF_LEN = 192;
static char s_line_buf[LINE_BUF_LEN];
static size_t s_line_len = 0;

static bool parse_mq7(const char *line, unsigned int *raw, float *voltage, float *ppm) {
    const char *p = strstr(line, "MQ7_SS:");
    if (!p) {
        return false;
    }
    p += strlen("MQ7_SS:");
    return (sscanf(p, " raw=%u, voltage=%f V, ppm=%f", raw, voltage, ppm) == 3);
}

static bool parse_sc8(const char *line, unsigned int *co2) {
    const char *p = strstr(line, "SC8_SS:");
    if (!p) {
        return false;
    }
    p += strlen("SC8_SS:");
    return (sscanf(p, " co2=%u ppm", co2) == 1);
}

static bool parse_gp2y(const char *line, unsigned int *raw, float *voltage, float *dust) {
    const char *p = strstr(line, "GP2Y_SS:");
    if (!p) {
        return false;
    }
    p += strlen("GP2Y_SS:");
    return (sscanf(p, " raw=%u, voltage=%f V, dust=%f mg/m3", raw, voltage, dust) == 3);
}

static bool parse_photo(const char *line, unsigned int *raw, float *voltage) {
    const char *p = strstr(line, "PHOTO_SS:");
    if (!p) {
        return false;
    }
    p += strlen("PHOTO_SS:");
    return (sscanf(p, " raw=%u, voltage=%f V", raw, voltage) == 2);
}

static bool parse_dht11(const char *line, float *temp_c, float *hum_pct) {
    const char *p = strstr(line, "DHT11_SS:");
    if (!p) {
        return false;
    }
    p += strlen("DHT11_SS:");
    return (sscanf(p, " temp=%f C, hum=%f %%", temp_c, hum_pct) == 2);
}

static void emit_mq7(float ppm) {
    Serial.printf("CO=%.2f ppm\r\n", ppm);
}

static void emit_sc8(unsigned int co2) {
    Serial.printf("CO2=%u ppm\r\n", co2);
}

static void emit_gp2y(float dust) {
    Serial.printf("PM2.5=%.3f mg/m3\r\n", dust);
}

static void emit_photo(float voltage) {
    Serial.printf("LIGHT=%.3f V\r\n", voltage);
}

static void emit_dht11(float temp_c, float hum_pct) {
    Serial.printf("TEMP=%.1f C, HUM=%.1f %%\r\n", temp_c, hum_pct);
}

static void handle_line(const char *line) {
    unsigned int raw = 0;
    unsigned int co2 = 0;
    float voltage = 0.0f;
    float ppm = 0.0f;
    float dust = 0.0f;
    float temp_c = 0.0f;
    float hum_pct = 0.0f;

    if (parse_mq7(line, &raw, &voltage, &ppm)) {
        emit_mq7(ppm);
        return;
    }

    if (parse_sc8(line, &co2)) {
        emit_sc8(co2);
        return;
    }

    if (parse_gp2y(line, &raw, &voltage, &dust)) {
        emit_gp2y(dust);
        return;
    }

    if (parse_photo(line, &raw, &voltage)) {
        emit_photo(voltage);
        return;
    }

    if (parse_dht11(line, &temp_c, &hum_pct)) {
        emit_dht11(temp_c, hum_pct);
        return;
    }
}

static bool ensure_bt_connected() {
    if (SerialBT.connected()) {
        return true;
    }

    static uint32_t last_attempt_ms = 0;
    const uint32_t now = millis();
    if ((now - last_attempt_ms) < 2000u) {
        return false;
    }
    last_attempt_ms = now;

    Serial.println("BT: connecting to HC-05...");
    bool ok = false;
    if (HC05_USE_MAC && HC05_MAC[0] != '\0') {
        ok = SerialBT.connect(HC05_MAC);
    } else {
        ok = SerialBT.connect(HC05_NAME);
    }

    Serial.println(ok ? "BT: connected" : "BT: connect failed");
    return ok;
}

void setup() {
    Serial.begin(115200);

    // true => master mode (ESP32 initiates the SPP connection to HC-05)
    if (!SerialBT.begin("ESP32_GATEWAY", true)) {
        Serial.println("BT: BluetoothSerial begin failed");
    }
    if (HC05_PIN[0] != '\0') {
        // ESP32 core requires pin length explicitly.
        SerialBT.setPin(HC05_PIN, (uint8_t)strlen(HC05_PIN));
    }

    // Try immediately once.
    (void)ensure_bt_connected();
}

void loop() {
    if (!ensure_bt_connected()) {
        delay(10);
        return;
    }

    while (SerialBT.available() > 0) {
        int c = SerialBT.read();
        if (c < 0) {
            break;
        }
        if (c == '\n') {
            s_line_buf[s_line_len] = '\0';
            handle_line(s_line_buf);
            s_line_len = 0;
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (s_line_len < (LINE_BUF_LEN - 1)) {
            s_line_buf[s_line_len++] = (char)c;
        } else {
            // overflow: drop line
            s_line_len = 0;
        }
    }
}
