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
- **DHT11 temperature & humidity sensor**
- **10kΩ pull-up resistor** (Data pin → VCC)
- **USB-UART converter** (FTDI)
- Breadboard &/or Jumper wires

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
### Pull-up Resistor
- **Required** for open-drain operation
- Typical value: **10 kΩ** (between Data & VCC)
- 
The DHT11’s data pin is open-drain (also called open-collector).
The sensor can only pull the line LOW.

When it “releases” the line, something else must pull it HIGH.
This where the pull-up resistor comes in.

**How It Works**
When no device is pulling the line LOW, the pull-up resistor gently pulls it to a stable *HIGH level (logic 1)*.

This prevents the line from **floating** (unreliable, noisy readings).

**Built-In vs External Resistor**
Many DHT11 modules already have a 10 kΩ pull-up resistor soldered on the PCB.

If you’re using a bare DHT11 sensor , you must add an external pull-up resistor:

*Value*: 4.7 kΩ to 10 kΩ (10 kΩ is standard)*

**Tip:** For long wires or high-noise environments, use 4.7 kΩ instead of 10 kΩ.
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
Full Handshake + Bit Transmission:

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
The Dallas 1-Wire protocol, now commonly known as 1-Wire, is a** low-speed, low-power communication protocol** developed by Dallas Semiconductor (now Maxim Integrated) for connecting devices like sensors and memory chips to a master microcontroller over a single data line. 
It's a master-slave protocol where the master initiates all communication, and slaves respond. 
The protocol allows for data and power transmission over the same wire, making it efficient for applications with limited wiring.

## Potential Issues
1. Power Supply Issue
- MCU powered through ST-LINK flashing device → only 2.8 V at DHT11 (insufficient).
- Solution: Provide proper 3.3 V / 5 V to DHT11.
2. Timing Issues:
- The DHT11 is very timing sensitive.
- If the pin stays LOW or HIGH for even 1 micro seconds beyond the threshhold limit. It won't respond. Resulting in a Timeout .
- Solution: Ensure MCU sets line LOW before handshake wait
4. Bit Shifting Error:
- Received data contained incorrect temperature values (e.g., 88°C).
- Cause: Assigning a byte of memory directly to a single bit position.
- Solution: Shift bits correctly — insert into the rightmost position of buffer and shift left for each new bit.

Debug Checklist — Start Signal Timing

Create a HAL_TIM_GET_COUNTER instance to measure the duration of voltage pulses from DHT11.
If initial handshake signals match expected timing, proceed to bit reading.

      
