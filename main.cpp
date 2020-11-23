#include "mbed.h"
#include "ultrasonic.h"
#include "LSM9DS1.h"
#include "motordriver.h"

void dist(int distance){
    printf("Distance %d mm\r\n", distance);
}

ultrasonic sens1(p5, p6, .1, 1, &dist);
ultrasonic sens2(p7, p8, .1, 1, &dist);
ultrasonic sens3(p9, p10, .1, 1, &dist);
ultrasonic sens4(p11, p12, .1, 1, &dist);
Serial pc(USBTX, USBRX);
Motor left(p21, p25, p26, 1);
Motor right(p22, p23, p24, 1);
Serial blue(p28, p27);

int main(){
    LSM9DS1 9dof(p13, p14, 0xD6, 0x3C);
    9dof.begin();
    if(!9dof.begin()){
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    9dof.calibrate();
    sens1.startUpdates();
    sens2.startUpdates();
    sens3.startUpdates();
    sens4.startUpdates();

    while(1){
        sens1.checkDistance();
        sens2.checkDistance();
        sens3.checkDistance();
        sens4.checkDistance();

        imu.readTemp();
        imu.readMag();
        imu.readGyro();

        pc.printf("gyro: %d %d %d\n\r", imu.gx, imu.gy, imu.gz);
        pc.printf("accel: %d %d %d\n\r", imu.ax, imu.ay, imu.az);
        pc.printf("mag: %d %d %d\n\n\r", imu.mx, imu.my, imu.mz);
        wait(1);
    }
}



