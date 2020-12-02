# RC Robot
Author: Vincent Wang

## Introduction
This is my final project for ECE 4180 at Georgia Institute of Technology. The goal of my project was to create a bluetooth controlled RC car that could return to the user using an inertial measurement unit (IMU) while avoiding obstacles with ultrasonic sensors.

![robot](/media/robot.jpeg)

## Parts List
* [mbed LPC1768](https://os.mbed.com/platforms/mbed-LPC1768/)
* [Adafruit Bluefruit LE UART Friend - Bluetooth Low Energy (BLE)](https://www.adafruit.com/product/2479#technical-details)
* 2 x [HC-SR04 Ultrasonic Sonar Distance Sensor](https://www.adafruit.com/product/3942)
* [SparkFun Motor Driver - Dual TB6612FNG](https://www.sparkfun.com/products/14450)
* [SparkFun 9DoF Sensor Stick](https://www.sparkfun.com/products/13944)
* [Hobby Gearmotor - 140 RPM (Pair)](https://www.sparkfun.com/products/13302)
* [Shadow Robot Chasis](https://www.sparkfun.com/products/13301)
* [Wheel - 65mm (Rubber Tire, Pair)](https://www.sparkfun.com/products/13259)
* Breadboard, wires, 4 x AA batteries

## Schematic
![schematic](/media/schematic.png)

## Wiring
mbed | Bluefruit LE
------------ | -------------
gnd | gnd
VU (5V) | Vin
nc | MOD, RTS, DFU
gnd | CTS
p13 | RXI
p14 | TXO

mbed | Ultrasonic Sensors
------------ | -------------
gnd | gnd
VU (5V) | Vcc
p5, p7 | Trig
p6, p8 | Echo

mbed/motors | Motor Driver
------------ | -------------
gnd | gnd
external 5V | Vm
Vout | Vcc
Vout | STBY
p21 | PWMA
p22 | PWMB
p25 | AI1
p26 | AI2
p29 | BI1
p30 | BI2
Motor 1 + | AO1
Motor 1 - | AO2
Motor 2 + | BO1
Motor 2 - | BO2

mbed | 9DOF IMU
------------ | -------------
gnd | gnd
Vout | Vdd
p27 | SCL
p28 | SDA

## Instructions
1. Download the Bluefruit Connect app
1. Power on the robot and let it spin around to calibrate the magnetometer
1. In the Bluefruit app, connect to the Adafruit Bluefruit LE, go to Controller, then Control Pad.
1. The direction arrows control where the robot is going.
1. Press 1 to set/reset the origin. 
1. Press 2 to return to the origin.

## Software
Software is hosted here on Github as well as in the mbed compiler at: https://os.mbed.com/users/vwang314/code/RC_Robot/
There are some issues with libraries not being pulled in when pulling code from Github in the mbed online compiler so a .zip file has been included that can be imported into the mbed online compiler.

The main thread handles Bluetooth control to make the robot behave like a basic RC car. A second thread handles the ultrasonic sensors. A third thread is included for measuring location with the IMU but it is not used in the current version since the IMU was too inaccurate and odometry provided better results.

imu.cpp and imu.h contain the functions needed to return to the origin. There are two methods implemented: one using IMU and the other using odometry. The IMU method uses the accelerometer and kinematics to update the robots position by sampling the accelerometer very frequently to find velocity and therefore displacement. Odometry is handled in the main thread where a timer is used to record how long the robot travels in a particular direction. This time is then split into time driving in the x and y direction using the IMU's magnetic heading. There are two very similar functions used to return to the origin. One uses IMU input, distance travelled in the x and y direction, the other using odometry, time travelled in the x and y direction. An important note regarding all calculations: the x and y axis are the orientation of Cartesian coordinates, facing east and north. In trigonometry, 0 degrees starts at east and goes counterclockwise but magnetic heading starts at north and goes clockwise; this is the equivalent of the x and y axis being swapped. Therefore, a lot of the calculations where sin traditionally is used for the y component and cos for x have been flipped where sin is for x and cos for y and tan is x/y instead of y/x.

collision.cpp and collision.h contain the functions for the ultrasonic sensors. It contains the Thread function to keep sampling ultrasonic sensor readings as well as the functions the sensors call when the distance changes.

## Demo
https://youtu.be/mdOVJAlg2k8
