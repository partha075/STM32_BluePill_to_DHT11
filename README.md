# STM32_BluePill_to_DHT11
Read Temperature and Humidity Values from a DHT11-Sensor to the STM32F103C8T6 BluePill Board. 
# STM32 DHT11 Temperature & Humidity Sensor Interface

This project implements precise communication with a **DHT11 temperature and humidity sensor** using an **STM32F103C8T6** (Blue Pill) microcontroller.  
The firmware is written in **C** using **STM32CubeIDE** and uses **TIM3 microsecond delay routines** for accurate signal timing.

---

## 📌 Features
- Full DHT11 communication protocol implementation
- Uses **hardware timer (TIM3)** for microsecond precision
- UART output for debugging
- Configurable pull-up resistor value for reliable communication
- Clean modular code (`dht11.c`, `delay.c`)

---

## 🛠 Hardware Used
- **STM32F103C8T6 (Blue Pill)**
- DHT11 temperature & humidity sensor
- 10kΩ pull-up resistor (Data pin → VCC)
- USB-UART converter (FTDI / CH340)
- Breadboard & jumper wires

---

## 📡 Pin Connections

| STM32 Pin | Function   | Connection       |
|-----------|-----------|------------------|
| PA1       | DHT11 Data| Sensor Data Pin  |
| PA9       | UART TX   | FTDI RX          |
| GND       | Ground    | Sensor GND       |
| 3.3V      | Power     | Sensor VCC       |

---

## 🧩 Communication Logic

The **DHT11** uses a **single-wire bidirectional half-duplex protocol** where the host (STM32) initiates the transaction and the sensor responds.

### Initialization Sequence
1. **MCU Output LOW** (≥ 18 ms) – wake-up signal to DHT11  
2. **MCU Output HIGH** (20–40 µs) – release the line  
3. **DHT11 Response**:  
   - Pulls line LOW for ~80 µs  
   - Pulls line HIGH for ~80 µs  

### Data Transmission (40 bits)
- Each bit starts with a **50 µs LOW pulse** from the sensor
- Followed by a HIGH pulse:
  - **26–28 µs HIGH** → `0`
  - **~70 µs HIGH** → `1`

**Data Frame:**

A Pull-up 

### 3️⃣ Pull-up Resistor
- **Required** for open-drain operation
- Typical value: **10 kΩ** (between Data & VCC)
- Lower values (4.7 kΩ) improve noise immunity on longer cables

---

## ⏱ STM32 Clock Configuration

To achieve **microsecond precision**, the project configures TIM3 as follows:

- **System Clock (HCLK)**: 72 MHz  
- **Timer Prescaler**: `72 - 1` → 1 MHz timer tick (1 µs resolution)  
- **Auto-reload register (ARR)**: Set dynamically based on delay requirements  
- Delay functions:  
  ```c
  void delay_us(uint16_t us);
  void delay_ms(uint16_t ms);
  
## ⏱ STM32 and DHT11 Timing Diagram

## Full Handshake + Bit Transmission:

arduino
Copy
Edit
Host (STM32)                Sensor (DHT11)
│                           │
LOW  --------------------   │   (≥ 18 ms)
HIGH  ---                   │   (20–40 µs)
                            LOW ----  (80 µs)
                            HIGH ---- (80 µs)

Bit Transmission:
LOW ----                    │   (50 µs)
HIGH --                     │   (~26–28 µs) → '0'
HIGH --------               │   (~70 µs) → '1'

## Communication Protocol
