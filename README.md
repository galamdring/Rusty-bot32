# Quadruped Robot Project Plan: ESP32-S3 & Rust

This document outlines the comprehensive plan for building your 12 DoF Quadruped Robot, integrating hardware, mechanical design, and firmware development with a focus on future AI capabilities.

---

### I. Project Vision & Goals

* **Robot Type:** Quadruped (4 legs), 12 Degrees of Freedom (DoF - 3 servos per leg).
* **Aesthetics:** Kame robot leg design combined with a more enclosed, sleek SpotMicro-like body, with a distinct sensor "head."
* **Control:** ESP32-S3 microcontroller, firmware written in Rust.
* **Awareness:** Initial awareness via front-facing ultrasonic sensors and a camera. Head will have 2 DoF (pan/tilt) for wider sensor coverage.
* **Future Goal:** Integrate AI/TinyML for advanced environmental perception and intelligent behavior.
* **Budget:** Minimal budget, maximizing functionality.

---

### II. Hardware Acquisition (Your Shopping List & Confirmations)

* **Servos (14 Total):**
    * **12 x MG90S Servos:** For the legs. Confirm you have 12.
    * **2 x MG90S Servos:** For the 2 DoF head (pan & tilt). You will need to acquire these if you don't have spares beyond the 12 for the legs.
    * **Action:** Verify the voltage tolerance of your MG90S servos, as some prefer 5V-6V, while others can handle 7.4V (2S LiPo). This impacts your battery choice.
* **Main Controller:**
    * **1 x ESP32-S3 Development Board:** (Your "basic one").
    * **Action:** Identify the **exact model/manufacturer** of your specific ESP32-S3 board. Get its pinout diagram and documentation. This is crucial for wiring the camera and other sensors correctly.
* **Servo Driver:**
    * **1 x PCA9685 16-channel 12-bit PWM Servo Driver Board:** Essential for controlling all 14 servos.
* **Power System:**
    * **1 x LiPo Battery (2S or 3S):** For powering the servos. Choose based on servo voltage tolerance and required capacity (e.g., 2000-4000mAh, 20C+).
    * **1 x LiPo Balance Charger:** Non-negotiable for safe LiPo charging.
    * **1 x LiPo Low Voltage Alarm/Buzzer:** For battery safety.
    * **1 x DC-DC Buck Converter Module (e.g., Mini-360 or LM2596 based):** To step down the LiPo voltage to power the ESP32-S3 (usually 5V or 3.3V) and the logic of the PCA9685.
* **IMU (Inertial Measurement Unit):**
    * **1 x MPU6050 Module:** For basic robot orientation and self-awareness.
* **Camera Module:**
    * **1 x OV2640 Camera Module:** Must be compatible with your specific ESP32-S3 board's camera interface (check pinout and connector type).
* **Environmental Sensors (Initial):**
    * **2 x HC-SR04 Ultrasonic Sensors:** For front-facing obstacle detection in the head.
* **Wiring:**
    * Confirm you have sufficient **Dupont jumper wires** (male-to-female, male-to-male, female-to-female).
    * Consider **thicker gauge wire** for main power lines from the battery to the servo power rail on the PCA9685 and the buck converter, due to high current draw.
    * Potentially **servo extension cables** if needed for leg wires.
* **Misc. Prototyping:**
    * **Breadboard:** (Confirmed you have).
    * **USB Cable:** For programming your ESP32-S3.

---

### III. Mechanical Design & 3D Printing

1.  **Chassis Design (Your Hybrid Vision):**
    * **Body:** Design a 3D printable main body that is sleek and enclosed (SpotMicro aesthetic) to house all electronics (ESP32-S3, PCA9685, LiPo, buck converter, IMU).
        * Include **access panels** for battery and component access.
        * Design **internal channels/clips** for cable management.
        * Consider **ventilation slots** for heat dissipation.
        * Ensure a **flat, central, and rigid mounting point** for the MPU6050 IMU.
    * **Head:** Design a separate 3D printable head unit.
        * Create **precise mounting holes/slots** for the OV2640 camera (top) and the two HC-SR04 ultrasonic sensors (front/eyes).
        * Design the internal structure to accommodate the two servos for **pan and tilt motion**.
        * Plan for **wire routing** from the head to the main body.
    * **Legs:** Design 4 sets of 3-DoF legs based on the **Kame robot leg geometry**.
        * Design **robust mounting points** on the main body for the hip servos of each leg.
        * Ensure all servo mounts are **snug and precise** for the MG90S servos.
    * **Overall Size & Weight:** Keep the entire robot **small and lightweight** (aim for under 300-400g total weight) to maximize performance with MG90S servos.
    * **Fasteners:** Design for your existing screws, nuts, and standoffs.
2.  **3D Printing:**
    * Print all chassis components using a durable filament like **PETG** for structural parts, and **PLA** for non-stressed components if desired to save cost.
    * Pay attention to **print orientation** for maximizing part strength.

---

### IV. Electronic Assembly & Wiring

1.  **Power System:**
    * Wire the LiPo battery to the **buck converter** (outputting 5V or 3.3V) and to the **power input of the PCA9685**.
    * Wire the buck converter's output to power the **ESP32-S3** and the **logic input of the PCA9685**.
    * Connect the **LiPo alarm**.
2.  **PCA9685 Wiring:**
    * Connect all 14 servos (12 legs, 2 head) to individual channels on the PCA9685.
    * Connect the PCA9685's I2C pins (SDA, SCL) to the corresponding I2C pins on your ESP32-S3.
3.  **Sensor Wiring:**
    * Wire the MPU6050 IMU to another I2C bus on your ESP32-S3 (or share with PCA9685 if addresses don't conflict, though separate is often cleaner).
    * Wire the OV2640 camera module to the designated camera interface pins on your ESP32-S3. This will require careful attention to the pinout.
    * Wire the two HC-SR04 ultrasonic sensors to dedicated GPIO pins on the ESP32-S3 (each needs a Trig and Echo pin).
4.  **Initial Power-Up & Testing:**
    * **Crucial:** Double-check all wiring before applying power. Incorrect wiring can damage components.
    * Test each power rail separately before connecting components where possible.

---

### V. Firmware Development (Rust on ESP32-S3)

1.  **Development Environment Setup:**
    * Install **`rustup`** and the Rust toolchain.
    * Install **`espup`** for ESP-IDF toolchain and Rust targets for ESP32.
    * Familiarize yourself with "The Rust on ESP Book" (`docs.esp-rs.org/book/`) – this is your primary resource.
    * Use the `esp-idf-template` for a new project, as it provides `std` support for features like Wi-Fi.
2.  **Core Driver Development (Rust Crates):**
    * **ESP32-S3 HAL (Hardware Abstraction Layer):** Use the appropriate `esp32s3-hal` crate to interact with the ESP32's peripherals (GPIO, I2C, PWM, UART).
    * **PCA9685 Driver:** Find or adapt an `embedded-hal` compatible Rust driver for the PCA9685.
    * **MPU6050 Driver:** Find or adapt an `embedded-hal` compatible Rust driver for the MPU6050.
    * **HC-SR04 Driver:** Implement a simple Rust driver for the ultrasonic sensors (trigger pulse, measure echo duration).
    * **OV2640 Camera Driver:** This will be more complex. Look for existing Rust crates or examples for interfacing the OV2640 with ESP32.
3.  **Servo & Head Control:**
    * Implement functions to set precise PWM values for all 14 servos via the PCA9685.
    * Develop **Inverse Kinematics (IK)** algorithms in Rust for the 3-DoF legs. This will be the core of your robot's movement.
    * Implement **Gait Generation:** Define and program different walking gaits (e.g., trot, walk) for the quadruped, sequencing the leg movements based on IK outputs.
    * Develop **Head Pan/Tilt Control:** Functions to move the head servos to specific angles.
    * Implement **Head Scanning Routines:** Functions to sweep the head horizontally and/or vertically for wider sensor coverage.
4.  **Sensor Data Acquisition:**
    * Read data from the MPU6050 (accelerometer, gyroscope).
    * Read distance data from the HC-SR04 sensors.
    * Capture image frames from the OV2640 camera.
5.  **Communication & Control Interface:**
    * Leverage the ESP32's Wi-Fi capabilities.
    * Implement a **simple HTTP web server (in Rust)** on the ESP32 to receive commands from a web browser (e.g., "move forward," "turn left," "scan head"). This acts as your backend control system for the robot.
    * Consider a **basic serial interface** for debugging and direct commands during development.
6.  **Core Robot Logic:**
    * Implement a main control loop that handles:
        * Reading sensor data (including the head's pan/tilt angles).
        * Processing incoming commands (from web server).
        * Executing gaits and movement commands.
        * Basic safety checks (e.g., low battery alarm).

---

### VI. Initial Testing & Debugging

1.  **Individual Component Testing:** Test each component (servos, PCA9685, IMU, ultrasonic, camera) individually with simple Rust code before integrating them.
2.  **Leg Calibration:** Calibrate each servo's "zero" position.
3.  **Kinematics Validation:** Test your IK solution by manually setting target foot positions and verifying the leg moves correctly.
4.  **Gait Testing:** Implement and test simple forward/backward/turning gaits.
5.  **Sensor Integration Testing:** Verify that sensor readings are accurate and make sense.
6.  **Head Movement Testing:** Test pan and tilt ranges, ensure smooth movement.

---

### VII. Future AI Integration (Beyond First Iteration)

1.  **Data Collection:** Start collecting sensor data (including camera frames and associated head angles) during robot operation.
2.  **TinyML Frameworks:** Explore using TensorFlow Lite for Microcontrollers (TFLite Micro) or Edge Impulse with Rust on ESP32-S3.
3.  **Model Training:** Train small machine learning models on a PC/cloud based on your collected data (e.g., for object detection, sound classification, simple environment categorization).
4.  **On-Device Inference:** Deploy the trained models to your ESP32-S3 and integrate their inference output into your robot's decision-making logic.
