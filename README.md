# **Rough Terrain Rover**
## _Six-wheeled rough terrain rover built with ESP32, LoRa communication, and FPV camera system._

This project is a six-wheeled rough terrain rover that was developed to demonstrate autonomous and manual navigation in challenging environments. The rover is built using an ESP32 development board as the main controller and integrates several sensors, actuators, and communication modules to achieve reliable performance. The design is focused on stability and adaptability, which is why a six-wheel configuration was chosen. Each of the six wheels is powered by a 12V DC motor, and the motors are controlled using three L298N motor driver modules. This provides enough torque and stability for the rover to move on uneven or rough surfaces.

A 2-axis gimbal mechanism is mounted on the rover, which holds both a VL53L0X Time-of-Flight (TOF) sensor and a First-Person View (FPV) VTX camera with a 5.8 GHz video transmitter. The gimbal is controlled by two micro servo motors that allow smooth panning and tilting. The TOF sensor is used to scan the terrain and detect obstacles, while the FPV camera provides a live video feed to the operator. The video feed is transmitted to a receiver that is connected to the laptop, allowing real-time monitoring of the rover’s environment.

For location tracking, the rover uses a NEO-6M GPS module. This module provides longitude and latitude values, which are then displayed on a live 2D map in the control application. The map allows the operator to visualize the rover’s movement and track its path over long distances. Since the project was designed for long-range control, LoRa communication modules were integrated to handle two-way communication between the rover and the laptop. LoRa ensures stable control and telemetry exchange over large distances, which is not possible with standard Wi-Fi or Bluetooth connections.

The rover is equipped with a headlight system that allows it to navigate in dark or low-light conditions. The headlight can be controlled remotely from the Python-based control application. This application is a complete control center for the rover and includes several important features. It has directional control buttons to drive the rover forward, backward, left, and right, as well as buttons to control the camera gimbal. A live video window displays the FPV camera feed, while a separate panel shows the rover’s position on a 2D map generated from GPS data. The application also displays LoRa communication logs, provides buttons to switch the headlight on and off, and allows the operator to toggle between manual and autonomous operation modes.

In manual mode, the operator directly controls all movements and camera operations. In autonomous mode, the rover relies on its TOF sensor to avoid obstacles and make navigation decisions on its own. This dual-mode capability makes the rover versatile for both experimental teleoperation and semi-autonomous missions. From a hardware perspective, the project includes an ESP32 microcontroller, six 12V DC motors, three L298N motor drivers, two micro servo motors for the gimbal, the VL53L0X TOF sensor, a NEO-6M GPS module, an FPV VTX camera with a 5.8 GHz transmitter and receiver, LoRa transceiver modules, a high-power LED headlight, and a custom-built chassis that can be either 3D printed or assembled with standard mechanical components. On the software side, the ESP32 firmware is developed in Arduino C++ and handles motor control, LoRa communication, GPS parsing, servo control for the gimbal, and obstacle avoidance logic. The Python control application is designed with a graphical interface, built using Tkinter or PyQt. OpenCV is used to display the live video feed from the FPV camera, and mapping libraries such as Folium or Matplotlib are used to render the rover’s GPS position on a live 2D map. The communication between the rover and the laptop is handled through serial LoRa modules, ensuring that commands and telemetry are transmitted efficiently.

The overall working principle of the rover is as follows: the operator connects both the LoRa receiver and FPV video receiver to the laptop. Once the Python application is launched, the operator can choose to control the rover manually using the GUI buttons or switch it into autonomous mode. In manual mode, commands are sent from the Python app through LoRa to the rover, which drives the motors and servos accordingly. The live camera feed is displayed in real-time, and the GPS coordinates are shown on the map. In autonomous mode, the rover scans the terrain using the TOF sensor and makes navigation decisions to avoid obstacles without human input.

* ### Features –
 #### 🛞 Six-Wheel Rough Terrain Design 
- Six **12V DC motors** controlled by **3 × L298N motor drivers**.  
- Independent wheel design for high stability.  
- Adaptable for rocks, uneven ground, and slopes. 
#### 🎥 2-Axis Camera Gimbal with FPVs
- Equipped with **2 × micro servos** for smooth pan/tilt control.  
- **FPV VTX Camera (5.8 GHz)** for live video feed.  
- Video receiver connects to laptop for real-time display in Python app.  
### 📡 LoRa Communication
- Long-distance wireless control via **LoRa transceivers**.  
- Provides **stable communication** between rover and base station.  
- All control commands and telemetry are exchanged through LoRa.
#### 📍 GPS Tracking 
- Uses **NEO-6M GPS module** to fetch **latitude and longitude**.  
- Python app displays rover position on a **2D live map**.  
- Enables long-range tracking of rover’s movement. 
#### 🌍 Terrain Detection with TOF Sensor
- **VL53L0X Time-of-Flight (TOF) sensor** mounted on gimbal.  
- Scans terrain ahead of the rover.  
- Provides distance data for **obstacle avoidance in auto mode**.  
#### 🔦 Headlight System
- High-power LED headlights.  
- Controlled via Python app.  
- Helps navigate in **dark or low-light environments**.  
#### 💻 Python Control Application 
The **Python GUI application** provides:  
- Direction control buttons for the rover.  
- Camera gimbal control (pan/tilt).  
- Live FPV video window.  
- Real-time **2D GPS map**.  
- LoRa communication logs.  
- Headlight ON/OFF controls.  
- Mode switching: **Manual ↔ Auto**.

* ### Project PCB Designs -
  
  [PCB_PCB_Rover_3_2025-02-07.pdf](https://github.com/user-attachments/files/22261948/PCB_PCB_Rover_3_2025-02-07.pdf)
  
  [PCB_PCB_Rover_Main_2025-02-15.pdf](https://github.com/user-attachments/files/22261955/PCB_PCB_Rover_Main_2025-02-15.pdf)
  
  [PCB_PCB_Rover_Power_2025-02-12.pdf](https://github.com/user-attachments/files/22261966/PCB_PCB_Rover_Power_2025-02-12.pdf)
  
  ![WhatsApp Image 2025-09-10 at 23 02 10_7a97ef36](https://github.com/user-attachments/assets/4de459d9-4b62-4591-b349-cb8fa47c67b5)
  ![WhatsApp Image 2025-09-10 at 23 02 10_1a066c32](https://github.com/user-attachments/assets/cbc573e5-18bd-4bdd-aba5-b2d781742ebb)
  ![WhatsApp Image 2025-09-10 at 23 02 10_6de0fa99](https://github.com/user-attachments/assets/2ee1f040-8a5e-44f3-b45d-0661aaf5e8e1)

* ### Project Demo Captures -

  ![WhatsApp Image 2025-09-10 at 23 02 06_301276b7](https://github.com/user-attachments/assets/2ed46847-3930-45a7-82f5-9dd920322a9a)
  ![WhatsApp Image 2025-09-10 at 23 02 06_797244d0](https://github.com/user-attachments/assets/1e3f838a-f35c-48f2-af55-dffc8113a5ab)
  ![WhatsApp Image 2025-09-10 at 23 02 07_90f354f5](https://github.com/user-attachments/assets/4ae280af-db81-423e-b1df-4549a4b6a087)
  ![WhatsApp Image 2025-09-10 at 23 02 07_cc22106a](https://github.com/user-attachments/assets/79a7c4bd-cd60-441b-b941-0cb0d3bde06b)
  ![WhatsApp Image 2025-09-10 at 23 02 09_586a1a76](https://github.com/user-attachments/assets/8cbfefa9-20f5-4a7e-bd76-f9475eee59bb)
  ![![WhatsApp Image 2025-09-10 at 23 02 11_39aed358](https://github.com/user-attachments/assets/91d1d2f1-8652-4461-a702-8eb01ed4ec61)
  WhatsApp Image 2025-09-10 at 23 02 09_2810a133](https://github.com/user-attachments/assets/faf87843-9f7c-4e4c-b6f5-be375d641ec7)
  ![WhatsApp Image 2025-09-10 at 23 50 11_7040d724](https://github.com/user-attachments/assets/11783d82-1303-46af-9c34-f8785fe5c17d)
  ![WhatsApp Image 2025-09-10 at 23 02 05_193c863a](https://github.com/user-attachments/assets/b068f3f4-624e-4f57-9e56-b3240874483f)

  * ### Project Demo Videos -
    
 https://github.com/user-attachments/assets/35ccc970-d5ea-46fc-bd60-64fc08a90199

 https://github.com/user-attachments/assets/0b12cd88-a34d-4f3a-9b5f-595c7ef7af0b
