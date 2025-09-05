Smart-AutoBots – WRO Future Engineers 2025

Table of Contents

1. Introduction


2. About WRO Future Engineers


3. Project Goals


4. Team Smart-AutoBots


5. Hardware Overview


6. Software Overview


7. Electronics & Wiring


8. Sensor Details


9. Motor Driver & Control


10. Algorithms and Code Structure


11. Repository Structure


12. Installation and Setup


13. How to Run the Project


14. Challenges & Strategy


15. Testing and Debugging


16. Safety Measures


17. Future Improvements




---

Introduction

Welcome to the Smart-AutoBots WRO 2025 repository, created for our participation in the World Robot Olympiad (WRO) Future Engineers category.
This project focuses on building a fully autonomous robot car that uses a Raspberry Pi 4, multiple sensors, and a custom control algorithm to complete tasks such as navigating obstacle courses, avoiding collisions, and responding dynamically to its environment.

The Smart-AutoBots team consists of passionate young engineers dedicated to designing, programming, and optimizing autonomous vehicles using cutting-edge robotics, computer vision, and artificial intelligence techniques.
Our goal is to compete at the WRO International Final 2025, showcasing a robot that demonstrates precision, intelligence, and innovation.


---

About WRO Future Engineers

The Future Engineers category of the WRO focuses on:

Autonomous vehicles

Sustainable transportation

Robotics for real-world challenges


Teams are tasked with designing robots that can:

Navigate complex environments

Detect and avoid obstacles

Identify specific targets or objects

Operate without human intervention


Key Requirements:

Robot must be fully autonomous after the match starts.

Modular design is encouraged to allow upgrades.

Code must be open-source and well-documented (hence this repository).

Safety features must be included to prevent damage or injuries.



---

Project Goals

Our project is focused on three core goals:

1. Innovation – Designing a robot car with unique sensor combinations and algorithms to stand out among competitors.


2. Performance – Ensuring high accuracy in obstacle detection, path planning, and object recognition.


3. Documentation – Providing complete open-source documentation, including:

Hardware schematics

Code

Algorithms

Setup instructions





---

Team Smart-AutoBots

We are Team Smart-AutoBots, a group of young roboticists passionate about technology and problem-solving.

Member Name	Role	Expertise

Reyansh Patel	Team Leader, Software Developer	Python, OpenCV, Raspberry Pi
Pratham	Hardware Specialist	Robotics hardware, wiring, motor control
Kanishk	Algorithm Developer	AI, control algorithms
Dhruv	Tester & Debugger	Field testing, tuning parameters


Mentor: Mr. A. B. Patel – Robotics Coach & Electronics Engineer


---

Hardware Overview

Our robot car uses a carefully selected combination of hardware components optimized for performance and reliability.

Main Components:

1. Raspberry Pi 4 (4GB RAM)

Acts as the main processing unit.

Handles computer vision, sensor data processing, and decision-making.



2. Pi Camera Module 3 Wide

Used for real-time video processing.

Wide-angle lens ensures better object detection coverage.



3. Ultrasonic Sensors (HC-SR04) – x2

Detects obstacles and measures distances on left and right sides.



4. Color Sensor (TCS34725)

Used to detect specific target colors like red, green, or blue.



5. Servo Motor (SG90)

Handles steering.



6. Motor Driver (TB6612FNG)

Controls two DC motors for forward and backward movement.



7. Power Source: Lithium-Ion Rechargeable Battery Pack

Provides stable 7.4V output with sufficient current.



8. Optional Expansion Modules

PCA9685: PWM signal management for multiple servos.

TCA9548: I2C multiplexer to handle multiple I2C devices.





---

Software Overview

We use Python as the main programming language due to its simplicity and extensive libraries for robotics.

Key Libraries:

OpenCV – Image processing and object detection.

RPi.GPIO – Raspberry Pi GPIO control.

picamera2 – Camera interface.

numpy – Mathematical operations and data handling.

adafruit_tcs34725 – Color sensor interface.

ultrasonic custom library – For HC-SR04 distance measurement.


Software Layers:

1. Sensor Layer – Handles raw data collection.


2. Processing Layer – Computer vision, filtering, and distance calculations.


3. Control Layer – Decision-making and command generation.


4. Actuator Layer – Executes commands to motors and servos.




---

Electronics & Wiring

Below is the wiring plan for the robot.

Component	GPIO Pin (Raspberry Pi)

Servo Motor (Steering)	GPIO 18 (PWM)
Left DC Motor	Motor A1, A2 via TB6612FNG
Right DC Motor	Motor B1, B2 via TB6612FNG
Ultrasonic Sensor (Left) Trigger	GPIO 23
Ultrasonic Sensor (Left) Echo	GPIO 24
Ultrasonic Sensor (Right) Trigger	GPIO 17
Ultrasonic Sensor (Right) Echo	GPIO 27
Color Sensor SDA	GPIO 2
Color Sensor SCL	GPIO 3


Wiring Diagram:

A visual wiring diagram is included in the /docs folder of the repository.


---

Sensor Details

Ultrasonic Sensor (HC-SR04)

Range: 2cm to 400cm

Used for:

Obstacle detection

Distance measurement



Color Sensor (TCS34725)

Detects RGB values.

Used to:

Identify challenge colors (red, green, blue).

Trigger specific robot actions.



Camera (Pi Camera 3 Wide)

Resolution: Up to 12MP

Uses OpenCV for:

Object detection

Face/shape tracking




---

Motor Driver & Control

We use the TB6612FNG Motor Driver because:

It provides bidirectional control for two motors.

Supports up to 1.2A continuous current.


TB6612FNG Pin	Connected To

PWMA	GPIO 12
AIN1	GPIO 20
AIN2	GPIO 21
PWMB	GPIO 13
BIN1	GPIO 19
BIN2	GPIO 26
STBY	GPIO 16



---

Algorithms and Code Structure

Our robot uses a modular code structure:

smartautobots-wro2025/
│
├── main.py                # Main control file
├── sensors/
│   ├── ultrasonic.py      # HC-SR04 control
│   ├── color_sensor.py    # TCS34725 control
│   └── camera.py          # Pi Camera functions
├── motors/
│   └── motor_driver.py    # TB6612FNG control
├── algorithms/
│   └── navigation.py      # Path planning and decision-making
├── utils/
│   └── logger.py          # Logging utilities
└── docs/
    └── wiring_diagram.png


---

Installation and Setup

1. Clone the Repository

git clone https://github.com/Smart-Autobots/smartautobots-wro2025.git
cd smartautobots-wro2025

2. Create Virtual Environment

python3 -m venv wro_env
source wro_env/bin/activate

3. Install Dependencies

pip install -r requirements.txt

4. Enable Raspberry Pi Interfaces

sudo raspi-config

Enable Camera

Enable I2C

Enable GPIO



---

How to Run the Project

Starting the Robot:

python3 main.py

The robot will:

1. Initialize sensors.


2. Start camera streaming.


3. Begin autonomous navigation.




---

Challenges & Strategy

We focus on two main challenge areas:

1. Obstacle Avoidance

Using dual ultrasonic sensors and camera vision to detect and avoid barriers.

Implementing PID control for smooth steering.



2. Color-Based Decision Making

Using the TCS34725 color sensor to identify target zones.

Triggering special actions such as stopping or turning.





---

Testing and Debugging

Testing is performed in three stages:

1. Unit Testing – Testing individual sensors and modules.


2. Integration Testing – Combining modules and ensuring communication.


3. Field Testing – Running the robot on actual challenge courses.



Debug logs are automatically saved in the /logs folder.


---

Safety Measures

Emergency stop button wired to GPIO 4.

Battery voltage monitoring to prevent deep discharge.

Overcurrent protection for motors.



---

Future Improvements

Add LIDAR sensor for more accurate mapping.

Implement advanced AI models for object detection.

Create a 3D simulation environment using Gazebo.
