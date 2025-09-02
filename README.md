Perfect — if you don’t have a wiring diagram image, a **clean wiring connections table** is just as professional (and sometimes clearer). I’ll add it as a section in your `README.md`, based on the pin mappings in your report.

Here’s the updated version with the wiring table included:

---

# Smart Safety Helmet 🦺🤖

## 📌 Overview

The **Smart Safety Helmet** is an **IoT-based wearable safety device** designed to enhance worker protection on construction sites. Powered by an **ESP32 microcontroller**, the helmet integrates **environmental and motion sensors** to detect hazards such as smoke, excessive heat, and falls, while providing **real-time alerts** via LED, buzzer, vibration, and cloud connectivity.

This project was developed as part of the course *Technology and Application of IoT* at Xiamen University Malaysia, and aligns closely with **Automation Development Engineering** practices by combining **embedded systems, robotics-inspired safety feedback, and industrial IoT communication**.

---

## 🚀 Key Features

* **Multi-sensor hazard detection**

  * MQ-2 smoke sensor (fire/gas hazards)
  * DHT11 temperature & humidity sensor (heat stress monitoring)
  * MPU-6050 accelerometer (fall/impact detection)
  * LDR (auto headlamp activation in low-light)

* **Real-time alerts**

  * RGB LED (status indication)
  * Piezo buzzer (audible warning)
  * Vibration motor (haptic feedback in noisy environments)

* **Automatic headlamp** for improved visibility in dark environments.

* **Cloud connectivity via MQTT** for remote supervision and faster emergency response.

* **Event-driven, non-blocking firmware** ensuring hazard detection under **400 ms** without blocking delays.

* **Security-first design** with **TLS 1.3 encrypted communication** and secure firmware updates.

---

## 🛠️ System Workflow

1. **Continuous monitoring** of temperature, smoke, light levels, and motion.
2. **Local alerts** via LED, buzzer, and vibration when a hazard is detected.
3. **Cloud notification** through MQTT, enabling supervisors to monitor workers in real time.
4. **User acknowledgement** with a long-press reset button to clear active alerts.
5. **Fail-safe recovery** ensures the helmet resumes safe monitoring after hazards are cleared.

---

## 🔧 Hardware Components

* **ESP32 development board** (Wi-Fi enabled MCU)
* **DHT11 sensor** (temperature & humidity)
* **MQ-2 sensor** (smoke/gas detection)
* **MPU-6050** (accelerometer + gyroscope for fall detection)
* **LDR + 1W LED** (auto headlamp control)
* **RGB LED** (status indicator)
* **Piezo buzzer** (audible alerts)
* **Coin vibration motor** (haptic feedback)
* **Push button** (alert reset)

---

## 📊 Wiring Connections

| Component               | ESP32 Pin(s)                 | Power & Ground                |
| ----------------------- | ---------------------------- | ----------------------------- |
| **DHT11 Temp/Humidity** | GPIO 17                      | 3.3 V, GND                    |
| **MQ-2 Smoke Sensor**   | GPIO 34 (DO)                 | 5 V, GND                      |
| **LDR Divider**         | GPIO 36 (ADC1\_CH0)          | 3.3 V, 10 kΩ → GND            |
| **Piezo Buzzer**        | GPIO 16                      | GND                           |
| **Vibration Motor**     | GPIO 32 (Gate)               | Motor + → 3.3 V, Source → GND |
| **1 W LED Headlamp**    | GPIO 33 (Gate)               | LED + → 3.3 V, Source → GND   |
| **RGB LED (Red/Green)** | GPIO 25 / GPIO 26            | Common GND                    |
| **Push Button (Clear)** | GPIO 4                       | GND                           |
| **MPU-6050 (I²C)**      | SDA → GPIO 21, SCL → GPIO 22 | 3.3 V, GND                    |

---

## 🖥️ Software Architecture

* **Event-driven firmware** written in Arduino (C++), deployed on ESP32.
* **Non-blocking design** using `millis()` for sensor polling and timing (no `delay()`), ensuring responsiveness.
* **Five subsystems running in parallel**:

  * Initialization & setup
  * Connectivity management (Wi-Fi + MQTT)
  * Sensor monitoring (light, smoke, temperature, motion)
  * Alert presentation (multimodal warnings)
  * Idle & recovery (return to safe standby)

👉 Full source code is available in [`SmartSafetyHelmet.ino`](SmartSafetyHelmet.ino).

---

## 📊 Performance

* Hazard detection within **\~400 ms**.
* Stable for a **full 8-hour work shift** on a rechargeable power bank.
* Cloud alerts published instantly via MQTT.
* Secure communication (TLS 1.3, WPA2-Enterprise Wi-Fi).

---

## 📸 Real Object Photos

![Uploading IMG_20250701_205637.jpg…]()


---

## 📂 Repository Structure

```
├── SmartSafetyHelmet.ino   # ESP32 firmware (Arduino)
├── Report.pdf              # Full project documentation
├── photos/                 # Folder for real object photos
└── README.md               # Project documentation
```

---

## 👥 Team Contributions

* **Tan Yi Zhao** — Project lead, hardware/software integration, system architecture.
* **Sim Wen Ken** — Hardware setup, ESP32 firmware co-development, diagrams.
* **Leong Ting Yi** — Cloud integration (MQTT), validation, workflow analysis.
* **Than You Siang** — Debugging, security/privacy documentation, final report.

---

## 📚 References

* ESP32 Documentation (Espressif Systems)
* MQTT Protocol Specification
* IoT Sensor Integration (DHT11, MQ-2, MPU6050 libraries)



Do you also want me to make a **small ASCII block diagram** of the system workflow (sensors → ESP32 → alerts/cloud) that you can add under the workflow section?
