#ifndef IMU
#define IMU

#define PI 3.141592653
#define DECLINATION -1.72 // Declination (degrees) in Germantown, TN

float calc_heading(float mx, float my, float mz);
void check_imu_func();
void return_to_origin(float x, float y);

#endif