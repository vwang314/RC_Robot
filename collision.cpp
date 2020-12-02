#include "collision.h"
#include "mbed.h"
#include "motordriver.h"
#include "ultrasonic.h"
#include "rtos.h"

extern bool moving;
extern Serial pc;
extern Motor left;
extern Motor right;
extern ultrasonic sens1;
extern ultrasonic sens2;
extern ultrasonic sens3;
extern ultrasonic sens4;

void dist1(int distance){
    if(distance < 40){
        moving = false;
        pc.printf("Sens1 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist2(int distance){
    if(distance < 40){
        moving = false;
        pc.printf("Sens2 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist3(int distance){
    if(distance < 40){
        moving = false;
        pc.printf("Sens3 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}
void dist4(int distance){
    if(distance < 40){
        moving = false;
        pc.printf("Sens4 Distance %d mm\r\n", distance);
        left.stop(0.5);
        right.stop(0.5);
    }
}

void check_dist_func(){
    while(1){
        sens1.checkDistance();
        //sens2.checkDistance();
        //sens3.checkDistance();
        sens4.checkDistance();
        Thread::wait(1);
    }
}