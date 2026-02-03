# IOT AIR MONITOR

STM32F103C8T6 firmware for multi‑sensor logging (UART debug) plus an ESP32 gateway
that filters logs and forwards only valid readings.

## Project layout

- `robot_agv/` — STM32 firmware (StdPeriph + custom drivers).
- `robot_agv/shared/Middle/sensor/` — sensor drivers.
- `robot_agv/shared/Middle/lcd/` — I2C LCD driver.
- `robot_agv/shared/Middle/debug/` — logger (USART2 + DMA) and HC‑05 RX parsing.
- `gateway/` — ESP32 Arduino sketches to filter STM32 logs.

## Build (STM32)

This repo includes a generated Makefile under `robot_agv/Debug/`.

```powershell
cd robot_agv\Debug
make all
```

You can also open the Eclipse/STM32CubeIDE project (`robot_agv/.project`).

## Sensor selection (main.c)

Select which sensors are active by setting `SENSOR_MODE_ACTIVE` in
`robot_agv/Src/main.c`.

Available modes:

- `SENSOR_MODE_MQ7`
- `SENSOR_MODE_SC8`
- `SENSOR_MODE_GP2Y`
- `SENSOR_MODE_PHOTO`
- `SENSOR_MODE_DHT11`
- `SENSOR_MODE_BOTH` (MQ7 + GP2Y)
- `SENSOR_MODE_PHOTO_DHT11` (Photodiode + DHT11)
- `SENSOR_MODE_ALL`

Default is `SENSOR_MODE_PHOTO_DHT11`.

## STM32 pin map (current code)

| Function | Signal | Pin |
| --- | --- | --- |
| MQ‑7 (ADC) | ADC1_CH0 | PA0 |
| SC8 CO2 (UART) | USART1_TX / RX | PA9 / PA10 |
| SC8 CO2 (PWM) | TIM3_CH2 input | PA7 |
| GP2Y1010 (ADC) | ADC1_CH3 | PA3 |
| GP2Y LED drive | GPIO | PA4 |
| Photodiode (ADC) | ADC1_CH6 | PA6 |
| DHT11 (1‑wire) | GPIO (open‑drain) | PA8 |
| Motor MOSFET | GPIO (active‑high) | PA5 |
| I2C LCD (PCF8574) | SCL / SDA | PB6 / PB7 |
| Debug UART | USART2_TX / RX | PA2 / PA3 |

Notes:
- **PA3 is shared** by GP2Y ADC and HC‑05 RX (USART2). Do not use both at the
  same time unless you move one signal to another pin.
- DHT11 needs a **pull‑up resistor** (4.7k–10k) on the data line.

## UART logs (STM32)

The firmware prints these lines via `debug_info()`:

```
MQ7_SS: raw=%u, voltage=%.3f V, ppm=%.2f
SC8_SS: co2=%u ppm
GP2Y_SS: raw=%u, voltage=%.3f V, dust=%.3f mg/m3
PHOTO_SS: raw=%u, voltage=%.3f V
DHT11_SS: temp=%.1f C, hum=%.1f %
```

## ESP32 gateway

### gateway/gateway.ino

- Connects to **HC‑05 over Bluetooth SPP** (ESP32 in master mode).
- Filters the STM32 logs above and prints only values:

```
CO=... ppm
CO2=... ppm
PM2.5=... mg/m3
LIGHT=... V
TEMP=... C, HUM=... %
```

Adjust `HC05_NAME`, `HC05_PIN`, or `HC05_MAC` inside the sketch.

### gateway/gateway2.ino

Example that forwards Bluetooth readings to Firebase (custom project; includes
hard‑coded Wi‑Fi/Firebase settings).

## LCD output

I2C LCD uses PCF8574 (default `0x27`). When `SENSOR_MODE_PHOTO_DHT11` is active,
the LCD shows temperature, humidity, and photodiode voltage. Otherwise it shows
CO/CO2/PM2.5.

## HC‑05 control (motor)

STM32 listens on USART2 RX for text commands (case‑insensitive):

- `FAN_ON` / `FAN_OFF`
- `Status : On` / `Status : Off`

On match, the MOSFET pin `PA5` is toggled.

