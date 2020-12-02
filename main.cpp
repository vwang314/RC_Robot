#include "mbed.h"
#include "rtos.h"
#include "collision.h"
#include "imu.h"

// I/O devices
Serial pc(USBTX, USBRX);
Serial blue(p13, p14);
Motor left(p21, p25, p26, 1);
Motor right(p22, p30, p29, 1);
LSM9DS1 imu(p28, p27, 0xD6, 0x3C);
ultrasonic sens1(p5, p6, .1, .2, &dist1);
ultrasonic sens2(p7, p8, .1, .2, &dist2);
Thread Check_Dist;
Thread Check_IMU;
bool originSet = false;            //boolean for whether in normal RC Car mode or return to origin mode

//variables for IMU method
bool moving = false;
float x_pos = 0.0;
float y_pos = 0.0;
float heading = 0.0;



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
    left.stop(0.2);
    right.stop(0.2);
    sens1.startUpdates();
    sens2.startUpdates();
    Check_Dist.start(check_dist_func);
    char bnum=0;
    char bhit=0;
    
    //variables for odometry/timer
    Timer t;
    t.start();
    float time;
    float x_time;
    float y_time;
    float start;
    float dir;

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
                                time = 0.0;
                                x_time = 0.0;
                                y_time = 0.0;
                                start = 0.0;
                                dir = 0.0;
                                t.reset();
                                //x_pos = 0.0;
                                //y_pos = 0.0;
                                //Check_IMU.start(check_imu_func);
                            } else {
                                //add release code here
                            }
                            break;
                        case '2': //number button 2,   return "home"
                            if (bhit=='1') {
                                if(originSet){
                                    //Check_IMU.terminate();
                                    //Check_Dist.terminate();     
                                    //moving = false;
                                    left.stop(0.2);
                                    right.stop(0.2);
                                    return_odometry(-x_time, -y_time);
                                    //return_to_origin(-x_pos, -y_pos);
                                    //Check_Dist.start(check_dist_func);
                                    //x_pos = 0.0;
                                    //y_pos = 0.0;
                                    time = 0.0;
                                    x_time = 0.0;
                                    y_time = 0.0;
                                    start = 0.0;
                                    dir = 0.0;
                                    t.reset();
                                    originSet = false;
                                }                            
                            } else {
                                //add release code here
                            }
                            break;
                        case '5': //button 5 up arrow
                            if (bhit=='1') {
                                if(originSet){
                                    dir = get_curr_heading();
                                    t.reset();
                                    t.start();
                                    start = t.read();
                                    moving = true;
                                }
                                left.speed(0.3);
                                right.speed(0.3);
                            } else {
                                if(originSet){
                                    time = t.read() - start;
                                    float dir2 = get_curr_heading();
                                    float i = (sin(dir2*PI/180) + sin(dir*PI/180) )/2; //since magnetic heading 0 degrees is at north, sin is x
                                    float j = (cos(dir2*PI/180) + cos(dir*PI/180) )/2; //cos is y
                                    dir = atan2(i,j)*180/PI;
                                    x_time += time*sin(dir/180*PI);
                                    y_time += time*cos(dir/180*PI);
                                    moving = false;
                                }
                                left.stop(0.2);
                                right.stop(0.2);
                                start = 0.0;
                                time = 0.0;
                                t.reset();
                                
                            }
                            break;
                        case '6': //button 6 down arrow
                            if (bhit=='1') {
                                if(originSet){
                                    dir = get_curr_heading();
                                    moving = true;
                                    start = t.read();
                                }
                                left.speed(-0.3);
                                right.speed(-0.3);
                            } else {
                                if(originSet){
                                    time = t.read() - start;
                                    float dir2 = get_curr_heading();
                                    float i = (sin(dir2*PI/180) + sin(dir*PI/180) )/2; //since magnetic heading 0 degrees is at north, sin is x
                                    float j = (cos(dir2*PI/180) + cos(dir*PI/180) )/2; //cos is y
                                    dir = atan2(i,j)*180/PI;
                                    x_time -= time*sin(dir/180*PI);
                                    y_time -= time*cos(dir/180*PI);
                                }
                                moving = false;
                                left.stop(0.2);
                                right.stop(0.2);
                                start = 0.0;
                                time = 0.0;
                                t.reset();  
                            }
                            break;
                        case '7': //button 7 left arrow
                            if (bhit=='1') {
                                left.speed(-0.2);
                                right.speed(0.2);
                            } else {
                                left.stop(0.2);
                                right.stop(0.2);
                            }
                            break;
                        case '8': //button 8 right arrow
                            if (bhit=='1') {
                                left.speed(0.2);
                                right.speed(-0.2);
                            } else {
                                left.stop(0.2);
                                right.stop(0.2);
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