#include "collision.h"

extern bool moving;
extern Serial pc;
extern Motor left;
extern Motor right;
extern ultrasonic sens1;
extern ultrasonic sens2;


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


void check_dist_func(){
    while(1){
        sens1.checkDistance();
        sens2.checkDistance();
        Thread::wait(1);
    }
}