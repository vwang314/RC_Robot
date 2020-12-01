# RC Robot
Author: Vincent Wang

## Introduction
This is the final project for ECE 4180 at Georgia Institute of Technology. The goal of this project was to create a bluetooth controlled RC car that could return to the user using an inertial measurement unit (IMU) while avoiding obstacles with ultrasonic sensors.

## Parts List
* [mbed LPC1768](https://os.mbed.com/platforms/mbed-LPC1768/)
* [Adafruit Bluefruit LE UART Friend - Bluetooth Low Energy (BLE)](https://www.adafruit.com/product/2479#technical-details)
* 4 x [HC-SR04 Ultrasonic Sonar Distance Sensor](https://www.adafruit.com/product/3942)
* [SparkFun Motor Driver - Dual TB6612FNG](https://www.sparkfun.com/products/14450)
* [SparkFun 9DoF Sensor Stick](https://www.sparkfun.com/products/13944)
* [Hobby Gearmotor - 140 RPM (Pair)](https://www.sparkfun.com/products/13302)
* [Shadow Robot Chasis](https://www.sparkfun.com/products/13301)
* [Wheel - 65mm (Rubber Tire, Pair)](https://www.sparkfun.com/products/13259)
* Breadboard, wires, 4 x AA batteries

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
p5, p7, p9, p11 | Trig
p6, p8, p10, p12 | Echo

mbed/motors | Motor Driver
------------ | -------------
gnd | gnd
external 5V | Vm
Vout | Vcc
Vout | STBY
p21 | PWMA
p22 | PWMB
p24 | AI1
p25 | AI2
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
