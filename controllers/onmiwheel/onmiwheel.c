/*
 * File:          onmiwheel.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

/*
 * macros
 */
#define TIME_STEP 64
#define PI 3.141592
#define OBSTACLE_DISTANCE 50.0


enum {
  Autonomous,
  Manual,
  Go,
  Turn,
  left,
  right,
  FreeWay,
  Obstacle
};

int  A = 65, S = 83, G = 71, W = 87;
int state;
double initial_angle_wheel1;

int checkForObstacles(WbDeviceTag distance_sensor) {
  double distance = wb_distance_sensor_get_value(distance_sensor);

  if (distance > OBSTACLE_DISTANCE)
    return FreeWay;
  else 
    return Obstacle;   
}

void goRobot(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_velocity(wheels[0], -velocity);
  wb_motor_set_velocity(wheels[1], velocity);
  wb_motor_set_velocity(wheels[2], 0);
}

void backRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], 0);
}

void leftRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 6);
}

void rightRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], -6);
}

void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

void turnLeft(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 6);
  wb_motor_set_velocity(wheels[1], 6);
  wb_motor_set_velocity(wheels[2], 6);
}

void turnRight(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], -6);
  wb_motor_set_velocity(wheels[2], -6);
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  double angle_wheel1 = wb_position_sensor_get_value(pos_sensor);
  double angle;

  angle = fabs(angle_wheel1 - initial_angle_wheel1);

  return angle;
}

/*
 * main
 */
int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  
  int key;
  float velocity;
  short int ds_state, robot_state = Go;
  float angle;

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
   WbDeviceTag wheels[2];
     wheels[0] = wb_robot_get_device("wheel1");
     wheels[1] = wb_robot_get_device("wheel2");
     wheels[2] = wb_robot_get_device("wheel3");
     
   wb_motor_set_position (wheels[0], INFINITY);
   wb_motor_set_position (wheels[1], INFINITY);
   wb_motor_set_position (wheels[2], INFINITY);
   
   WbDeviceTag encoder = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder, TIME_STEP);

   WbDeviceTag dist_sensor = wb_robot_get_device("DSENSOR_F1");
   wb_distance_sensor_enable(dist_sensor, TIME_STEP);
   
  /* 
   * main loop
   */
  while (wb_robot_step(TIME_STEP) != -1) {

     key = wb_keyboard_get_key();

     if (key == G)
       state = Autonomous;
     else if (key == W)
       state = Manual;
     else if (key == S){
       state = left;
       initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
     } else if (key == A){
         state = right;
         initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
     }
       
       
     if (state == Autonomous){
      if (robot_state == Go) {
        ds_state = checkForObstacles(dist_sensor);

        if (ds_state == FreeWay) {
          velocity = 8;
          goRobot(wheels, velocity);
          angle = wb_position_sensor_get_value(encoder);
          printf("Angle: %lf\n", angle);
        } else if (ds_state == Obstacle) {
            robot_state = Turn;
            stopRobot(wheels);
            initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
        }
      } else if (robot_state == Turn) {
          turnRight(wheels);
          angle = getAngleRobot(encoder);
          if (angle >= PI) {
            robot_state = Go;
            stopRobot(wheels);
          }
        }
     } else {
         if (key == WB_KEYBOARD_UP){
           velocity = 6;
           goRobot(wheels, velocity);
           angle = wb_position_sensor_get_value(encoder);
           printf("Angle: %lf\n", angle);
         } else if (key == WB_KEYBOARD_DOWN){
             backRobot(wheels);
             angle = wb_position_sensor_get_value(encoder);
             printf("Angle: %lf\n", angle);
         } else if (key == WB_KEYBOARD_LEFT){
             leftRobot(wheels);
             angle = wb_position_sensor_get_value(encoder);
             printf("Angle: %lf\n", angle);
         } else if (key == WB_KEYBOARD_RIGHT){
             rightRobot(wheels);
             angle = wb_position_sensor_get_value(encoder);
             printf("Angle: %lf\n", angle);
         } else if (state == left){
             turnLeft(wheels);
             angle = getAngleRobot(encoder);
             if (angle >= 0.4*PI) {
               robot_state = Go;
               stopRobot(wheels);
             }
         } else if (state == right){
             turnRight(wheels);
             angle = getAngleRobot(encoder);
             if (angle >= 0.4*PI) {
               robot_state = Go;
               stopRobot(wheels);
             }
         } else {
             stopRobot(wheels);
         }
           
       }
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
