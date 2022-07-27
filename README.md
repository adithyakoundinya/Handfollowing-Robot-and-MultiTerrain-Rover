# ros_esp_mr
Course workflow for ROS ESP32 interface in a mobile robot

## Quick Overview

We have designed 2 rovers with 5 modes collectively, 2 of which work over WiFi and 3 of which work over BlueTooth. The modes are as follows:

* BlueTooth Control using mobile app
* Hand-Following Mode
* WiFi Control using web-app
* ROS control via WiFi
* Path Memorization and Mimicry

## Rover Design and Hardware

The rover is designed such that is has multi-terrain off-roading capability through a 6-wheel drive chassis having a unique design. 

![Rover](https://github.com/[AryamanDubey00]/[ros_esp_mr]/blob/[img]/image.png?raw=true)

## HandFollowing Mode 

### Arduino Bluetooth Controlled Car

Before uploading the code you have to install the necessary library. AFMotor Library https://learn.adafruit.com/adafruit-motor-shield/library-install 

After downloading the library open Arduino IDE >> go to sketch >> Include Libray >> ADD. ZIP Libray >> Select the downloaded ZIP File >> Open it >> Done

Now You Can Upload the Code without any problem but make sure the bt module isn't connected with Arduino while uploading code

Note - Disconnect the Bluetooth Module before hiting the upload button otherwise you'll get compilation error message.


