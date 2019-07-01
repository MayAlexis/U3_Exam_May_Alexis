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
#include <webots/keyboard.h>

#include <stdio.h>
#include <math.h>

/*
 * macros
 */
#define TIME_STEP 64
#define PI 3.141592


enum {
  Autonomous,
  Manual,
};

int  A = 65, S = 83, G = 71, W = 87;
int state;

void goRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], -6);
  wb_motor_set_velocity(wheels[1], 6);
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
/*
 * main
 */
int main(int argc, char **argv)
{
  int key;

  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);

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
   
  /* 
   * main loop
   */
  while (wb_robot_step(TIME_STEP) != -1) {

    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
     key = wb_keyboard_get_key();

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_differential_wheels_set_speed(100.0,100.0);
     */
     if (key == G)
       state = Autonomous;
     else if (key == W)
       state = Manual;
       
       
       
     if (state == Autonomous){
       printf("Autonomous mode \n");
     } else {
         if (key == WB_KEYBOARD_UP){
           goRobot(wheels);
         } else if (key == WB_KEYBOARD_DOWN){
             backRobot(wheels);
         } else if (key == WB_KEYBOARD_LEFT){
             leftRobot(wheels);
         } else if (key == WB_KEYBOARD_RIGHT){
             rightRobot(wheels);
         } else if (key == S){
             turnLeft(wheels);
         } else if (key == A){
             turnRight(wheels);
         } else {
             stopRobot(wheels);
         }
           
       }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
