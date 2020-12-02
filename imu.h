#ifndef IMU
#define IMU

#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"
#include "motordriver.h"

#define PI 3.141592653
#define DECLINATION -1.72 // Declination (degrees) in Germantown, TN

float calc_heading(float mx, float my, float mz);
float get_curr_heading();
void return_odometry(float x, float y);
void check_imu_func();
void return_to_origin(float x, float y);

#endif