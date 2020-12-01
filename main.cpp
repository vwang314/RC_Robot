#include "mbed.h"
#include "rtos.h"
#include "ultrasonic.h"
#include "LSM9DS1.h"
#include "motordriver.h"
#include "collision.h"
#include "imu.h"

Serial pc(USBTX, USBRX);
Serial blue(p13, p14);
Motor left(p21, p25, p26, 1);
Motor right(p22, p30, p29, 1);
LSM9DS1 imu(p28, p27, 0xD6, 0x3C);
bool moving = false;
bool originSet = false;
float x_pos = 0.0;
float y_pos = 0.0;
float heading = 0.0;
ultrasonic sens1(p5, p6, .1, .2, &dist1);
ultrasonic sens2(p7, p8, .1, .2, &dist2);
ultrasonic sens3(p9, p10, .1, .2, &dist3);
ultrasonic sens4(p11, p12, .1, .2, &dist4);
Thread Check_Dist;
Thread Check_IMU;


//Main thread with bluetooth control
int main(){
    imu.begin();
    if(!imu.begin()){
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    imu.calibrate(1);
    left.speed(0.2);
    right.speed(-0.2);
    imu.calibrateMag(0);
    left.stop(0.5);
    right.stop(0.5);
    sens1.startUpdates();
    sens2.startUpdates();
    sens3.startUpdates();
    sens4.startUpdates();
    //Check_Dist.start(check_dist_func);
    char bnum=0;
    char bhit=0;

    while(1){
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data packet
                bnum = blue.getc(); //button number
                bhit = blue.getc(); //1=hit, 0=release
                if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    switch (bnum) {
                        case '1': //number button 1,  set "home" position
                            if (bhit=='1') {
                                //add hit code here
                                originSet = true;
                                x_pos = 0.0;
                                y_pos = 0.0;
                                Check_IMU.start(check_imu_func);
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2,   return "home"
                            if (bhit=='1') {
                                if(originSet){
                                    Check_IMU.terminate();
                                    //Check_Dist.terminate();     
                                    moving = false;
                                    left.stop(0.5);
                                    right.stop(0.5);
                                    return_to_origin(-x_pos, -y_pos);
                                    //Check_Dist.start(check_dist_func);
                                    originSet = false;
                                }                            
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow
                            if (bhit=='1') {
                                moving = true;
                                left.speed(0.3);
                                right.speed(0.3);
                            } else {
                                moving = false;
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '6': //button 6 down arrow
                            if (bhit=='1') {
                                moving = true;
                                left.speed(-0.3);
                                right.speed(-0.3);
                            } else {
                                moving = false;
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '7': //button 7 left arrow
                            if (bhit=='1') {
                                left.speed(-0.2);
                                right.speed(0.2);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                left.speed(0.2);
                                right.speed(-0.2);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        Thread::wait(1);
    }
}