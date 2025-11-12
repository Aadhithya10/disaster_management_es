# Smart Disaster Management System using STM32F401

## 📖 Overview
This project is a **multi-hazard detection system** designed to identify gas leaks, fires, and earthquakes in real time.  
It uses the **STM32F401 microcontroller** as the main controller and integrates multiple sensors with Bluetooth communication for remote alerts.

## ⚙️ Features
- 🔥 Flame detection using **IR flame sensor**
- 💨 Gas leak detection using **MQ-2 gas sensor**
- 🌎 Vibration/earthquake detection using **MPU6050 accelerometer**
- 📡 Real-time Bluetooth alerts via **HC-05 module**
- 🔔 Local alerts using **LEDs and buzzer**

## 🧩 Hardware Components
- STM32F401 Black Pill
- MQ-2 Gas Sensor
- Flame Sensor
- MPU6050 Accelerometer & Gyroscope
- HC-05 Bluetooth Module
- Buzzer and LEDs

## 🧠 Technical Details
- Language: **Embedded C**
- IDE: **STM32CubeIDE**
- Communication Interfaces: **ADC, I²C, UART, PWM**
- Alerts via **local buzzer/LED** and **Bluetooth messages**

## 🧪 Working Principle
1. Sensors continuously monitor environment.
2. Data processed by STM32F401.
3. If threshold exceeded (gas, flame, vibration), alert is triggered.
4. Alerts sent locally and via Bluetooth.

## 📁 Repository Contents
