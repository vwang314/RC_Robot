#include "mbed.h"
#include "ultrasonic.h"
#include "LSM9DS1.h"
#include "motordriver.h"

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
Serial pc(USBTX, USBRX);
Motor left(p21, p25, p26, 1);
Motor right(p22, p30, p29, 1);
Serial blue(p13, p14);

int main(){
    left.speed(0.5);
    wait(1);
    left.stop(0.5);
    right.speed(0.5);
    wait(1);
    right.stop(0.5);
    wait(1);
    left.speed(-0.5);
    wait(1);
    left.stop(0.5);
    right.speed(-0.5);
    wait(1);
    right.stop(0.5);

    LSM9DS1 dof(p28, p27, 0xD6, 0x3C);
    dof.begin();
    if(!dof.begin()){
        pc.printf("Failed to communicate with LSM9DS1.\n");
    }
    dof.calibrate();
    wait(10);
    sens1.startUpdates();
    sens2.startUpdates();
    sens3.startUpdates();
    sens4.startUpdates();

    while(1){
        sens1.checkDistance();
        sens2.checkDistance();
        sens3.checkDistance();
        sens4.checkDistance();

        dof.readTemp();
        dof.readMag();
        dof.readGyro();

        pc.printf("gyro: %d %d %d\n\r", dof.gx, dof.gy, dof.gz);
        pc.printf("accel: %d %d %d\n\r", dof.ax, dof.ay, dof.az);
        pc.printf("mag: %d %d %d\n\n\r", dof.mx, dof.my, dof.mz);
        wait(1);
    }
}



