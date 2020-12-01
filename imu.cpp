#include "imu.h"
#include "mbed.h"
#include "LSM9DS1.h"
#include "rtos.h"
#include "motordriver.h"

extern bool moving;
extern float x_pos;
extern float y_pos;
extern float heading;
extern LSM9DS1 imu;
extern Serial pc;
extern Motor left;
extern Motor right;
extern Thread Check_IMU;

//Function to convert magnetic readings in (x,y,z) to magnetic heading in degrees
float calc_heading(float mx, float my, float mz){
    mx = -mx;
    float mag_heading;
    if (my == 0.0)
        mag_heading = (mx < 0.0) ? 180.0 : 0.0;
    else
        mag_heading = atan2(mx, my)*360.0/(2.0*PI);
    mag_heading -= DECLINATION; //correct for geo location
    if(mag_heading>180.0) mag_heading = mag_heading - 360.0;
    else if(mag_heading<-180.0) mag_heading = 360.0 + mag_heading;
    else if(mag_heading<0.0) mag_heading = 360.0  + mag_heading;

    //pc.printf("Magnetic Heading: %f degress\n\r",mag_heading);
    return mag_heading;
}

void check_imu_func(){   //check imu to find current location and heading
    x_pos = 0.0;
    y_pos = 0.0;
    float x_dist = 0.0;
    float y_dist = 0.0;
    heading = 0.0;
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
            imu.readMag();
            heading = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
            pc.printf("x pos: %9f\n\r", x_pos);
            pc.printf("y pos: %9f\n\r", y_pos);
            pc.printf("Magnetic Heading: %f degress\n\n\r", heading);
            Thread::wait(1);
            time = t.read() - start;
            start = t.read();          
            //kinematics
            x_vel2 = x_vel1 - imu.calcAccel(imu.ax)*9.80665*time;
            y_vel2 = y_vel1 + imu.calcAccel(imu.ay)*9.80665*time;
            x_dist = (x_vel1+x_vel2)/2*time;        //average velocity
            y_dist = (y_vel1+y_vel2)/2*time;
            //convert x and y distance to true +x (east) and true +y (north) 
            x_pos = x_pos + cos((360-heading)*PI/180)*x_dist - sin((360-heading)*PI/180)*y_dist;
            y_pos = y_pos + sin((360-heading)*PI/180)*x_dist + cos((360-heading)*PI/180)*y_dist;
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

void return_to_origin(float x, float y){    //function to return to origin given origin's location relative to the robot
    //origin is at (x,y) and robot is at (0,0)
    imu.readMag();
    float dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));   //robot current heading
    
    /*float origin_dir = atan2(y,x)*180/PI;       //angle of origin
    if(90 < origin_dir && origin_dir < 180){origin_dir -= 360;}                 //convert trig angle (0 at east, 90 at north) to magnetic heading (0 at north, 90 at east)
    origin_dir = 90 - origin_dir;
    
    if(dir < origin_dir){     //face direction of origin
        while(dir < origin_dir){
            left.speed(0.2);
            right.speed(-0.2);
            imu.readMag();
            dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        }
    } else if(dir > origin_dir){
        while(dir > origin_dir){
            left.speed(-0.2);
            right.speed(0.2);
            imu.readMag();
            dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        }
    }
    left.stop(0.5);
    right.stop(0.5);

    x_pos = 0.0;
    y_pos = 0.0;
    moving = false;
    Check_IMU.start(check_imu_func);
    while( (abs(x_pos) < abs(x)) || (abs(y_pos) < abs(y)) ){
        moving = true;
        left.speed(0.3);
        right.speed(0.3);
    }
    Check_IMU.terminate();
    moving = false;
    left.stop(0.5);
    right.stop(0.5); */
      
    if(x<0) {    //if origin is to left of robot
        if(dir > 270.0){       //turn to face left
            while(dir > 270.0){
                left.speed(-0.2);
                right.speed(0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        } else {
            while(dir < 270.0){
                left.speed(0.2);
                right.speed(-0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        }
        left.stop(0.5);
        right.stop(0.5);
        moving = true;
        Check_IMU.start(check_imu_func);  
        while(x_pos > x){       //keep going left until at the x value of the origin
            left.speed(0.3);
            right.speed(0.3);
        }
        moving = false;
        left.stop(0.5);
        right.stop(0.5);
    } else if(x>0){     //if origin is to the right
        if(dir < 90.0){       //turn to face right
            while(dir < 90.0){
                left.speed(0.2);
                right.speed(-0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        } else {
            while(dir > 90.0){
                left.speed(-0.2);
                right.speed(0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        }
        left.stop(0.5);
        right.stop(0.5);
        moving = true;   
        while(x_pos < x){       //keep going right until at x-value of origin
            left.speed(0.3);
            right.speed(0.3);
        }
        moving = false;
        left.stop(0.5);
        right.stop(0.5);        
    }
    if(y_pos > y){       //if origin is below robot
        if(dir < 180.0){       //turn to face down
            while(dir < 180.0){
                left.speed(0.2);
                right.speed(-0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        } else {
            while(dir > 180.0){
                left.speed(-0.2);
                right.speed(0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        }
        left.stop(0.5);
        right.stop(0.5);
        moving = true;   
        while(y_pos > y){       //keep going down until at the origin
            left.speed(0.3);
            right.speed(0.3);
        }
        moving = false;
        left.stop(0.5);
        right.stop(0.5);
    } else if(y_pos < y){
        if(dir+180 < 360.0){       //turn to face up
            while(dir < 180){
                left.speed(-0.2);
                right.speed(0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        } else {
            while(dir+180 > 360.0){
                left.speed(0.2);
                right.speed(-0.2);
                imu.readMag();
                dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
                pc.printf("heading: %9f \n\r", dir);
            }
        }
        left.stop(0.5);
        right.stop(0.5);
        moving = true;   
        while(y_pos < y){       //keep going up until at the origin
            left.speed(0.3);
            right.speed(0.3);
        }
        Check_IMU.terminate();
        moving = false;
        left.stop(0.5);
        right.stop(0.5);
    }
}