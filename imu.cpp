#include "imu.h"

extern bool moving;
extern float x_pos;
extern float y_pos;
extern float heading;
extern LSM9DS1 imu;
extern Serial pc;
extern Motor left;
extern Motor right;
extern Thread Check_IMU;

/******************************************************************
* NOTE: all trig functions are used differently in this section; magnetic heading starts at 0 facing north and goes clockwise
*       whereas in trig 0 starts at east and goes counter clockwise; an important realization is that the x and y axis of
*       magnetic heading is opposite of the Cartesian axis so sin is used for x, cos for y, and tan for x/y
******************************************************************/

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

//function to sample and average heading readings and returns current heading
float get_curr_heading(){
    float i;  //for converting angles to complex coordinates
    float j;
    for(int ii=0; ii<10; ii++){   //average readings for current heading
        imu.readMag();
        float temp_dir = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));   //robot current heading
        pc.printf("Robot is facing: %f \n\r", temp_dir);
        i += sin(temp_dir*PI/180); //since magnetic heading 0 degrees is at north, sin is x
        j += cos(temp_dir*PI/180); //cos is y
    }
    i /= 10;
    j /= 10;
    float dir = atan2(i,j)*180/PI;
    pc.printf("Avg heading is: %f \n\r", dir);       
    return dir;
}

/****************************************
  Function to return to origin using odometry/timer
****************************************/

void return_odometry(float x, float y){
    //origin is at (x,y) and robot is at (0,0)
    pc.printf("Origin is at: %9f %9f \n\r", x, y);
    float dir = get_curr_heading(); 
    pc.printf("Avg heading is: %f \n\r", dir);       
    
    float origin_dir = atan2(x,y)*180/PI;       //angle of origin, arctan(x/y) 
    if(origin_dir < 0) { origin_dir += 360.0; }
    pc.printf("Origin is at heading: %f \n\n\r", origin_dir);
    
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
    left.stop(0.2);
    right.stop(0.2);

    left.speed(0.3);
    right.speed(0.3);
    wait( sqrt(x*x + y*y) );   //travel that distance for length of time
    left.stop(0.2);
    right.stop(0.2);
}


/****************************************
  Functions to return to origin using IMU
****************************************/

void check_imu_func(){   //check imu to find current location and heading
    x_pos = 0.0;
    y_pos = 0.0;
    float x_accel = 0.0;
    float y_accel = 0.0;
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
            imu.readMag();
            heading = calc_heading(imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
            imu.readAccel();
            pc.printf("imu raw a: %9f\n\r", imu.calcAccel(imu.ay));
            x_accel = imu.calcAccel(imu.ay)*sin(heading/180*PI);
            y_accel = imu.calcAccel(imu.ay)*cos(heading/180*PI);
            pc.printf("x accel: %9f\n\r", x_accel);
            pc.printf("y accel: %9f\n\r", y_accel);
            pc.printf("x pos: %9f\n\r", x_pos);
            pc.printf("y pos: %9f\n\r", y_pos);
            pc.printf("Magnetic Heading: %f degress\n\n\r", heading);
            Thread::wait(1);
            time = t.read() - start;
            start = t.read();
            //kinematics
            x_vel2 = x_vel1 + x_accel*9.80665*time;
            y_vel2 = y_vel1 + y_accel*9.80665*time;
            x_pos = x_pos + (x_vel1+x_vel2)/2*time;        //average velocity
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

void return_to_origin(float x, float y){    //function to return to origin given origin's location relative to the robot
    //origin is at (x,y) and robot is at (0,0)
    pc.printf("Origin is at: %9f %9f \n\r", x, y);
    float dir = get_curr_heading(); 
    pc.printf("Avg heading is: %f \n\r", dir);       
    
    float origin_dir = atan2(x,y)*180/PI;       //angle of origin, tan-1(x/y) 
    if(origin_dir < 0) { origin_dir += 360.0; }
    pc.printf("Origin is at heading: %f \n\n\r", origin_dir);
    
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
    left.stop(0.2);
    right.stop(0.2);

    x_pos = 0.0;
    y_pos = 0.0;
    moving = false;
    Check_IMU.start(check_imu_func);
    while( (abs(x_pos) < abs(x)) || (abs(y_pos) < abs(y)) ){
        moving = true;
        left.speed(0.3);
        right.speed(0.3);
        wait( sqrt(x*x + y*y) );
    }
    Check_IMU.terminate();
    moving = false;
    left.stop(0.2);
    right.stop(0.2);
}