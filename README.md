

### 🏡 Autonomous Greenhouse System 

#### 🌟 **Objective:**

To build a smart, energy-efficient greenhouse that autonomously monitors and controls environmental factors like temperature, humidity, and light intensity to optimize plant growth, reduce manual intervention, and enhance agricultural productivity.

---

### 🧠 **System Overview:**

The Autonomous Greenhouse System is a microcontroller-based automated solution that integrates multiple environmental sensors and actuators. It intelligently manages climate parameters using real-time data from sensors, making decisions to turn on/off appliances like motors, fans, and lights for a sustainable plant-growing environment.

---

### ⚙️ **Key Features:**

| Feature                                   | Description                                                                                                                   |
| ----------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| 🌡️ **Temperature & Humidity Monitoring** | Uses a DHT11 sensor to constantly measure the temperature and humidity inside the greenhouse.                                 |
| 🌞 **Light Intensity Control**            | Utilizes an LDR (Light Dependent Resistor) with ADC to sense ambient light and control artificial lighting using LEDs.        |
| 🔒 **Security System**                    | A passkey-based access system ensures that only authorized persons can interact with the control system.                      |
| 📟 **LCD Display**                        | A 16x2 LCD provides real-time data such as temperature, humidity, light level, and system status.                             |
| 🎛️ **Keypad Interface**                  | A 4x4 keypad is used for passkey entry to unlock or configure the system.                                                     |
| 💨 **Motor & Fan Control**                | A motor driver controls ventilation or cooling fans based on environmental needs.                                             |
| 🔁 **Relay Control**                      | A relay module controls external devices like heaters or water pumps.                                                         |
| 🎰 **Random Access Code**                 | On reset, a random number is shown on a 7-segment display; user must enter its reverse as the passkey to activate the system. |
| 🧠 **Fully Automated**                    | Once access is granted, the system autonomously monitors and reacts to environmental conditions.                              |

---

### 🖥️ **Hardware Components:**

| Component                      | Purpose                                                             |
| ------------------------------ | ------------------------------------------------------------------- |
| **STM32F405 Microcontroller**  | Core processing unit managing sensor data and controlling outputs.  |
| **DHT11 Sensor**               | Measures temperature and humidity.                                  |
| **LDR Sensor with ADC**        | Detects light intensity and converts analog values for LED control. |
| **16x2 LCD**                   | Displays real-time system information.                              |
| **Keypad (4x4)**               | User input interface for password/passkey.                          |
| **7-Segment Display (TM1637)** | Displays random number for passkey authentication.                  |
| **Motor Driver (TB6612FNG)**   | Drives motors/fans for climate control.                             |
| **Relay Module**               | Controls high-voltage devices like water pumps or heaters.          |
| **LED Bar Graph**              | Visually represents light levels based on LDR readings.             |

---

### 🔄 **System Flow:**

1. **Initialization:**

   * LCD shows a welcome message.
   * 7-segment displays a random 2-digit number.

2. **Security Authentication:**

   * User must input the reverse of the number via keypad.
   * If correct, the system activates.

3. **Live Monitoring Mode:**

   * DHT11 reads temperature and humidity continuously.
   * LDR senses ambient light.
   * If light is low, LEDs turn ON via GPIO.
   * If temperature/humidity is abnormal, motor/relay devices are activated.

4. **Autonomous Operation:**

   * System runs without user intervention, adjusting devices in real-time.

---

### ✅ **Benefits:**

* Reduces manual effort in greenhouse management.
* Maintains optimal conditions for plant growth.
* Improves crop yield and energy efficiency.
* Scalable and customizable for different greenhouse sizes.

---

### 👨‍💻 **Contributors:**

* **Aishwarya Singhal** – System Design, Integration, and Project Coordination
* **Ankit Saxena** – Core Coding and Debugging
* **Aviral Khare** – Core Coding and Debugging
* **Bharat** – Hardware Assembly, Circuit Schematic, and Physical Implementation
* **Muskan** – Hardware Assembly, Circuit Schematic, and Physical Implementation

---


