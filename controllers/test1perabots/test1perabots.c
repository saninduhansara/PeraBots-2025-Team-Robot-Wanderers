#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <webots/camera.h>


#define TIME_STEP 32

// Constants
#define DIST_PROPORTIONAL_CONST 0.034 / 2
#define INPUT_DISTANCE 10
#define ERROR_DIST 3.0
#define SPEED 6.0  // Adjust motor speed (Webots uses rad/s or % of maxVelocity)
#define MAX_SENSOR_VALUE 950
#define MAX_OF_SENSOR 800
#define MAX_DIFFERENCE 6
#define MAX_TURN_SPEED 2.0
#define CALIBRATION 0
#define MAX_ALIGN_ANGLE 5.0
#define COLLISION_DISTANCE 10.0



// PID constants
double kp_d = 0.5 ,kd_d = 0, ki_d = 0.009;


// State variables
float total_error =0;
float errorD = 0, previousErrorD = 0;
float integral = 0, derivative = 0;
float outputD = 0, outputA = 0;
float previousAngle = 0;

WbDeviceTag leftSensor1, leftSensor2, frontSensor, leftcorner;
WbDeviceTag leftMotor, rightMotor;

float read_sensor(WbDeviceTag sensor) {
  float value = wb_distance_sensor_get_value(sensor);
  if (value == 0.0 || value >= MAX_SENSOR_VALUE)
    value = MAX_OF_SENSOR;
  float distance = value * DIST_PROPORTIONAL_CONST;
  return distance;
  //printf("distance: %lf",distance);
}


float current_distance(float read1, float read2) {
  return (read1 + read2) / 2.0;
}


int check_region(float read1, float read2){
  float distance = current_distance(read1, read2);
  if(fabs(distance-INPUT_DISTANCE)>ERROR_DIST){
    if(distance > INPUT_DISTANCE){
        return -1; 
      }else{
        return 1; 
        }
    }else{
      return 0; //inside region
      }
  }

void set_motor_speeds(float speedL, float speedR) {
  wb_motor_set_velocity(leftMotor, speedL);
  wb_motor_set_velocity(rightMotor, speedR);
}
//////



void load_pid_constants() {
  FILE *file = fopen("pid_constants.txt", "r");
  if (file != NULL) {
    fscanf(file, "Kp: %lf\nKi: %lf\nKd: %lf\n", &kp_d, &ki_d, &kd_d);
    fclose(file);
    printf("Loaded PID constants: Kp = %lf, Ki = %lf, Kd = %lf\n", kp_d, ki_d, kd_d);
  } else {
    printf("PID file not found, using defaults.\n");
    kp_d = 1.0;
    ki_d = 0.0;
    kd_d = 0.0;
  }
  
}

/////////
void reach_distance(float distance1, float distance2) {
  float currentDistance = current_distance(distance1, distance2);
  errorD = currentDistance - INPUT_DISTANCE;
  
  
  float dt = TIME_STEP / 1000.0;  // TIME_STEP is in ms
  integral += errorD * dt;
  derivative = (errorD - previousErrorD) / dt;
  outputD = kp_d * errorD + ki_d * integral + kd_d * derivative;
  total_error += fabs(errorD);
  printf("%lf\n",errorD);

  //steps++;

  float speedL = SPEED - outputD;
  float speedR = SPEED + outputD;

  
  
  if ((speedL - speedR) > MAX_DIFFERENCE) {
    speedL = SPEED + MAX_DIFFERENCE;
    speedR = SPEED - MAX_DIFFERENCE;
    //printf("\nSPEDDL&R:%lf, %lf",speedL,speedR);
  } else if ((speedL - speedR) < -MAX_DIFFERENCE) {
    speedL = SPEED - MAX_DIFFERENCE;
    speedR = SPEED + MAX_DIFFERENCE;
    //printf("\nSPEDDL&R:%lf, %lf",speedL,speedR);
  } 
  
  
  
  
   //
  
  if (speedL >6.27){
    speedL = 6.27;
  }
  if (speedR >6.27){
    speedR = 6.27;
  }
  //

  //printf("speedL:%lf, speedR:%lf, errorD:%lf ,derivative:%lf ,integral:%lf, outputD:%lf, previousErrorD:%lf\n ",speedL,speedR,errorD,derivative,integral,outputD,previousErrorD);
  //printf("kp:%lf, ki:,%lf , kd:%lf\n",kp_d, ki_d, kd_d);
  set_motor_speeds(speedL, speedR);
 
  
  
  FILE *error_file = fopen("error.txt", "a");
  if (error_file) {
    double time = wb_robot_get_time();  // Get simulation time
    fprintf(error_file, "Time: %.2f, Error: %.4f\n", time, errorD);

    fclose(error_file);
   // printf("Total error written to error.txt: %.4f\n", total_error);
  }
  
}

bool check_collision() {
  float frontDistance = read_sensor(frontSensor);
  //printf("frontDistance= %lf\n",frontDistance);
  return frontDistance < COLLISION_DISTANCE;
}

int main() {
  wb_robot_init();
  
  


  // Initialize devices
  leftSensor1 = wb_robot_get_device("lf");
  leftSensor2 = wb_robot_get_device("lb");
  frontSensor = wb_robot_get_device("fs");
  leftcorner = wb_robot_get_device("lc");



  wb_distance_sensor_enable(leftSensor1, TIME_STEP);
  wb_distance_sensor_enable(leftSensor2, TIME_STEP);
  wb_distance_sensor_enable(frontSensor, TIME_STEP);
  wb_distance_sensor_enable(leftcorner, TIME_STEP);

  leftMotor = wb_robot_get_device("left wheel motor");
  rightMotor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);

  wb_motor_set_velocity(leftMotor, 0.0);
  wb_motor_set_velocity(rightMotor, 0.0);
  

  while (wb_robot_step(TIME_STEP) != -1) {
    load_pid_constants(); 
    float d1 = read_sensor(leftSensor1);
    float d2 = read_sensor(leftSensor2);

    int region = check_region(d1, d2);
    float frontDistance = read_sensor(frontSensor);
    float lcdistance = read_sensor(leftcorner);

    printf("Region:%d\n",region);
    
    //printf("d1:%lf,d2:%lf",d1,d2);
    float align_angle = fabs(d2 - d1);
    //printf(" align_angle:%lf\n", align_angle);
    
    
    //////
    
    

  
    //////////////
    
    if (check_collision()) {
      set_motor_speeds(SPEED + MAX_DIFFERENCE, SPEED - MAX_DIFFERENCE);
      //printf("\niwwwwi");
      
    } else if (d1>13.0 && d2>13.0 && frontDistance>8.0 && (lcdistance>13.59 &&lcdistance<13.61)) {
        wb_motor_set_velocity(leftMotor, SPEED);
        wb_motor_set_velocity(rightMotor, SPEED);
        
        //printf("lcdistance:%lf",lcdistance);
      
      
    } else if (align_angle > MAX_ALIGN_ANGLE) {
          reach_distance(d1, d2);
          //printf("%lf/n , %lf \n, %lfmmmm\n", d1, d2,frontDistance);
        
    }else if (region==1) {
      wb_motor_set_velocity(leftMotor, SPEED);
      wb_motor_set_velocity(rightMotor, 0);
    }else {
          reach_distance(d1, d2);
          //printf("%lf/n , %lf \n, %lfqqqqqq\n", d1, d2,frontDistance);
          //printf("lcdistance:%lf\n",lcdistance);
        }
      }
   
     
  wb_robot_cleanup();
  return 0;
}



