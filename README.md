# Quick Overview

A summer internship project done together by Aryaman Dubey and Sanjay K N of PES University in the Center for IoT under the Guidance of Dr. Adithya B.

We have designed 2 rovers with 5 modes collectively, 2 of which work over WiFi and 3 of which work over BlueTooth. The modes are as follows:

* BlueTooth Control using mobile app
* Hand-Following Mode
* WiFi Control using web-app
* ROS control via WiFi
* Path Memorization and Mimicry

# HandFollowing Robot

### Features

* Hand Following
* Remote Control Through Mobile App
* Accelerometer Control
* Gesture Control

### Hardware Components

![Handfollow Rover](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/handfollow1.jpeg)

### Components

1. 4-WHEEL CHASSIS (WITH MOTORS & BATTERY PACK)
2. ARDUINO UNO
3. ARDUINO CABLE
4. L293D MOTOR SHIELD
5. ULTRASONIC SENSOR
6. IR SENSORS x 2
7. SET OF JUMPERWIRES 
8. SWITCH FOR THE BATTERY

### Assembly

1.	Solder wires to the motors.
2.	Build the chassis & mount the motors onto it.
3.	Connect the wires from the motors to the L293D motor driver
4.	Mount the Arduino board, HC-05 module, switch, battery pack and the sensors as shown in the pictures.
5.	Connect the motor driver to the Arduino.
6.	Connect the power supply to a switch & connect the terminals from the switch on to the L293D motor driver 
7.	Connect the sensors & the HC-05 module to the analog pins on the motor driver using jumper wires.

![Handfollow Rover](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/handfollow2.jpeg)
![Handfollow Rover](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/handfollow3.jpeg)

### Setup

*	Download an Arduino ide for editing and uploading the code onto the board.
*	Install the required libraries and compile the code.
*	Upload the code to Arduino via the Arduino Uno cable.
*	Install the free “Bluetooth RC Controller” Application.

### Operating Instructions

*	Open the Applications and connect to the HC-05 Bluetooth module.
*	Drive the robot using arrow keys.
*	Press Parking Light button on the Application to put the robot into handfollow mode.
*	Open settings and press on Gesture control for controlling your robot using gestures

### Interface

![Interface for Bluetooth](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/btinter.jpeg)

### Notes

Before uploading the code you have to install the necessary library. AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install 

After downloading the library open Arduino IDE >> go to sketch >> Include Libray >> ADD. ZIP Libray >> Select the downloaded ZIP File >> Open it >> Done

Now You Can Upload the Code without any problem but make sure the bt module isn't connected with Arduino while uploading code

Note - Disconnect the Bluetooth Module before hiting the upload button otherwise you'll get compilation error message.

# Multi-Terrain Rover with WiFi, BlueTooth, ROS, Path Memorization/Mimicry 

### Rover Design and Hardware

The rover is designed such that is has multi-terrain off-roading capability through a 6-wheel drive chassis having a unique design. 

![Rover](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/image.png)

The design has a pair of rotatable front wheels which help in navigating off-road uneven terrain and handling verticality.

### Hardware Components

* 6-WHEEL ROVER CHASSIS (WITH MOTORS)
* ESP 32 DOIT DEVKIT V1(WIFI & BLUETOOTH MODULE)
* ESP32 CABLE
* 12V BATTERY PACK (for powering the motors)
* 10,400mAh POWER BANK (for powering the ESP 32)
* SET OF JUMPERWIRES 
* SWITCH FOR THE BATTERY
* L298N MOTOR DRIVERS x 3(for 6 motors)
* HARD WIRES FOR CONNECTING THE MOTORS TO THE DRIVERS
* BREAD BOARD MINI

### Setup

INSTRUCTIONS: 

1.	Solder wires to the motors.
2.	Build the chassis & mount the motors on to it.
3.	Connect the wires from the motors to the L298N motor drivers
4.	Mount a bread board, switch, power bank, 12v power supply and the motor drivers on the chassis.
5.	Fix your ESP32 on the bread board.
6.	Connect the motor driver pins in parallel to the ESP 32 on the bread board
7.	Connect the 12v power supply to a switch & connect the terminals from the switch on to the bread board
8.	Connect the Voltage pins and the ground pins in parallel with the terminals from the switch
9.	Power up the ESP32 by connecting it to the power bank

## Bluetooth Control Mode

### Bluetooth Setup and Interface

*	Install the free “Arduino Joystick” Application.
*	Connect to the ESP32 board when ready.

![App Interface Main](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/app1.jpeg)
![App Interface Setup](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/app2.jpeg)

Upload the BT + ROS code from this repo found in the [final codes](https://github.com/AryamanDubey00/ros_esp_mr/tree/main/ros_esp_mr/esp_src/Final%20Codes%20for%205%20modes) subfolder

## WiFi Control 

### Setup and Interface

1. Download an Arduino ide for editing and uploading the code onto the board.
2. Go to preferences and add [this url](https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json) in the additional boards manager URLs.
3. Go to the boards manager on your ide and install the latest esp 32 board package.
4. Select your ESP32 board. We use the “DOIT ESP32 DEVKIT V1” model and have chosen that.
5. Install the required libraries and compile the code given for WiFi rover in our Final Codes subdirectory.
6. Upload the code and connect to the wifi network  “C-IoT wifi rover” 
7. Open up your web browser and enter the URL “192.168.4.1”
8. A custom-made Interface will be available for driving the rover.

![App Interface](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/WiFiapp.jpeg)

## ROS Control via WiFi

ROS is an open-source framework for developers to create SW for robots working on a publisher-subscriber model. 

### Setup

1. Set up a Linux Ubuntu environment, whether that be in docker, VM, or as an OS on your system. The recommended distros are 18.04 (Bionic Beaver) or 20.04 (Focal Fossa). We use a VM with 20.04
2. Download the required version of ROS for your Ubuntu distro - for 18.04, it is ROS Melodic while for 20.04 it is ROS Noetic. We use Noetic. The detailed steps can be found [here.](http://wiki.ros.org/ROS/Installation)
3. Install the Rosserial package from [here](http://wiki.ros.org/rosserial). Ensure you are on the correct branch for your distro/version of ROS.
4. Install the Teleop twist keyboard package from [here](http://wiki.ros.org/teleop_twist_keyboard) to control the rover. Ensure you are on the correct branch for your distro/version of ROS.
5. ROS communicates to the robot over a common wifi network. Hence wifi has to be setup in the Code along with some other commands that are essential for the connection and communication between the rover and ROS.

Make sure that in the code, you replace the IP/WiFi settings with your own.

IPAddress server(IP address ex. “192.168.20.5”);
uint16_t serverPort = 11411;  //This can be left alone.
const char*  ssid = "your network name";
const char*  password = "your password";

You will need to set up at least 3 terminals to operate this mode, in this specific order.

1. roscore - roscore
2. rosserial - rosrun rosserial_python serial_node.py tcp (We use tcp, but you could try other protocols.)              
3. teleops - rosrun teleop_twist_keyboard teleop_twist_keyboard.py

![roscore](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/ros1.jpeg)
![rosserial](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/ros2.jpeg)
![teleops](https://github.com/AryamanDubey00/ros_esp_mr/blob/main/img/ros3.jpeg)

The code sets up a topic, /cmd_vel, to which the bot has subscribed and to which your system publishes every time you press a key in teleops. This is the basic communication methodology.

## Path Memorization and Mimicry

### Setup and Interface

*	Download an Arduino ide for editing and uploading the code onto the board.
*	Go to preferences and add https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json in the additional boards manager URLs.
*	Go to the boards manager on your ide and install the latest esp32 board package.
*	Select your ESP32 board model. We use the “DOIT ESP32 DEVKIT V1” board.
*	Install the required libraries and compile the code.
*	Upload the code onto the ESP32 board.

### Operating Instructions 

*	Open the “Arduino Joystick” application and connect to the ESP 32 board.
*	Drive the rover using arrow keys.
*	Press “X” to stop the rover.
*	Press the "Δ" button to command the rover to mimic the driven path.
*	Press “O” to reset and delete the recorded path to record a new path.




