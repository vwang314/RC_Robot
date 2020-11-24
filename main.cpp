#include "mbed.h"
#include "rtos.h"
#include "ultrasonic.h"
#include "LSM9DS1.h"
#include "motordriver.h"

Serial pc(USBTX, USBRX);
Serial blue(p13, p14);
Motor left(p21, p25, p26, 1);
Motor right(p22, p30, p29, 1);
LSM9DS1 imu(p28, p27, 0xD6, 0x3C);

void dist1(int distance){
    printf("Sens1 Distance %d mm\r\n", distance);
}
void dist2(int distance){
    printf("Sens2 Distance %d mm\r\n", distance);
}
void dist3(int distance){
    printf("Sens3 Distance %d mm\r\n", distance);
}
void dist4(int distance){
    printf("Sens4 Distance %d mm\r\n", distance);
}

ultrasonic sens1(p5, p6, .1, .2, &dist1);
ultrasonic sens2(p7, p8, .1, .2, &dist2);
ultrasonic sens3(p9, p10, .1, .2, &dist3);
ultrasonic sens4(p11, p12, .1, .2, &dist4);

void check_dist_func(void const *args){
    while(1){
        sens1.checkDistance();
        sens2.checkDistance();
        sens3.checkDistance();
        sens4.checkDistance();
        Thread::wait(1000);
    }
}

void check_imu_func(void const *args){
    while(1){
        imu.readTemp();
        imu.readMag();
        imu.readGyro();

        pc.printf("gyro: %d %d %d\n\r", imu.gx, imu.gy, imu.gz);
        pc.printf("accel: %d %d %d\n\r", imu.ax, imu.ay, imu.az);
        pc.printf("mag: %d %d %d\n\n\r", imu.mx, imu.my, imu.mz);
        Thread::wait(1000);
    }
}

int main(){  
    imu.begin();
    if(!imu.begin()){
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    imu.calibrate();
    wait(10);
    sens1.startUpdates();
    sens2.startUpdates();
    sens3.startUpdates();
    sens4.startUpdates();
    Thread Check_Dist(check_dist_func);
    Thread Check_IMU(check_imu_func);
    char bnum=0;
    char bhit=0;

    while(1){      
        if (blue.getc()=='!') {
            if (blue.getc()=='B') { //button data packet
                bnum = blue.getc(); //button number
                bhit = blue.getc(); //1=hit, 0=release
                if (blue.getc()==char(~('!' + 'B' + bnum + bhit))) { //checksum OK?
                    switch (bnum) {
                        case '1': //number button 1
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '3': //number button 3
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '4': //number button 4
                            if (bhit=='1') {
                                //add hit code here
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow
                            if (bhit=='1') {
                                left.speed(0.5);
                                right.speed(0.5);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '6': //button 6 down arrow
                            if (bhit=='1') {
                                left.speed(-0.5);
                                right.speed(-0.5);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '7': //button 7 left arrow
                            if (bhit=='1') {
                                left.speed(-0.5);
                                right.speed(0.5);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                left.speed(0.5);
                                right.speed(-0.5);
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
        Thread::wait(5);
    }
}