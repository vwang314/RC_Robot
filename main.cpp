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
bool originSet = false;
bool returnOrigin = false;
bool moving = false;

void dist1(int distance){
    if(distance < 50){
        moving = false;
        printf("Sens1 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist2(int distance){
    if(distance < 50){
        moving = false;
        printf("Sens2 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist3(int distance){
    if(distance < 50){
        moving = false;
        printf("Sens3 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist4(int distance){
    if(distance < 50){
        moving = false;
        printf("Sens4 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
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
        Thread::wait(1);
    }
}

void check_imu_func(void const *args){
    float x_pos = 0.0;
    float y_pos = 0.0;
    float x_vel1 = 0.0;
    float x_vel2 = 0.0;
    float y_vel1 = 0.0;
    float y_vel2 = 0.0;
    float time = 0.0;
    float start = 0.0;
    Timer t;
    t.start();
    while(1){
        if(moving){
            //to reduce noise, position is not constantly updated from IMU; only update position when motor is set to move
            imu.readAccel();
            pc.printf("x pos: %9f\n\r", x_pos);
            pc.printf("y pos: %9f\n\n\r", y_pos);
            Thread::wait(1);
            time = t.read() - start;
            start = t.read();
            x_vel2 = x_vel1 + imu.calcAccel(imu.ax)*9.80665*time;
            y_vel2 = y_vel1 - imu.calcAccel(imu.ay)*9.80665*time;
            x_pos = x_pos + (x_vel1+x_vel2)/2*time;
            y_pos = y_pos + (y_vel1+y_vel2)/2*time;
            x_vel1 = x_vel2;
            y_vel1 = y_vel2;
        } else {
            //robot is not moving so reset timer and set velocities to 0
            t.reset();
            x_vel1 = 0.0;
            x_vel2 = 0.0;
            y_vel1 = 0.0;
            y_vel2 = 0.0;
            start = 0.0;
            time = 0.0;
        }
    }
}

int main(){
    imu.begin();
    if(!imu.begin()){
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    imu.calibrate();
    wait(10);
    pc.printf("\n\rReady\n\n\r");
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
                                originSet = true;
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2
                            if (bhit=='1') {
                                //add hit code here
                                returnOrigin = true;
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
                                left.speed(-0.3);
                                right.speed(0.3);
                            } else {
                                left.stop(0.5);
                                right.stop(0.5);
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                left.speed(0.3);
                                right.speed(-0.3);
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