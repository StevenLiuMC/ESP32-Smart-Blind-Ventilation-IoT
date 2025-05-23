# ESP32-Smart-Blind-Ventilation-IoT

This project is an IoT-based smart blind and ventilation control system developed using the ESP32 development board. It combines environmental sensing and automated control to adjust indoor lighting and airflow.

## 🔧 Features

- Automatically opens/closes blinds or windows to regulate light and ventilation
- Makes decisions based on temperature, humidity, and light sensor data
- Supports remote monitoring and control (via Blynk Web platform)

## 🧱 Hardware Platform

- **Main Controller**: ESP32 development board
- **Sensors**: Temperature & humidity sensor, light sensor, IMU
- **Actuators**: Servo or motor to control blinds
- **Communication**: Wi-Fi

## How to use this project?
- Download this project as zip
- Unzip the zipped file and open the project with Arduino IDE or PlatformIO
- Upload the code to your ESP32 board (The project is divided into three components, each stored in a separate folder. You will need to upload the code from each folder to a different ESP32 board accordingly)
- Use Blynk Web version for remote control and data display
- **Notice**: The Blynk settings may vary and please create your own Blynk interface, this project can provide you with some ideas.

## 📄 License
This project is licensed under the MIT License. See the LICENSE file for details.
Contributions, suggestions, and issues are welcome!
