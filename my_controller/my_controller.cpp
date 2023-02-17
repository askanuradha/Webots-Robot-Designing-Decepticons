// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:
#define TIME_STEP 64
#define slider_speed 0.05
#define wheel_speed 1.5
#define gripper_speed 0.5
#define arm_speed 0.5
// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Motor *motor_f_r = robot->getMotor("motorfr");
  Motor *motor_b_l = robot->getMotor("motorbl");
  Motor *motor_f_l = robot->getMotor("motorfl");
  Motor *motor_b_r = robot->getMotor("motorbr");
  
  Motor *motor_f_r_t = robot->getMotor("motorfr_t");
  Motor *motor_b_l_t = robot->getMotor("motorbl_t");
  Motor *motor_f_l_t = robot->getMotor("motorfl_t");
  Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
  Motor *r_gear_motor = robot->getMotor("r_gear_motor");
  r_gear_motor->setPosition(INFINITY);
  r_gear_motor->setVelocity(0.0);
  Motor *l_gear_motor = robot->getMotor("l_gear_motor");
  l_gear_motor->setPosition(INFINITY);
  l_gear_motor->setVelocity(0.0);
  
  Motor *arm_motor = robot->getMotor("arm_motor");
  arm_motor->setPosition(INFINITY);
  arm_motor->setVelocity(0.0);
  
  //Motor *gripper_slider = robot->getMotor("gripper_slider");
  //gripper_slider->setPosition(INFINITY);
  //gripper_slider->setVelocity(0.0);
  
  Motor *arm_slider = robot->getMotor("arm_slider");
  arm_slider->setPosition(INFINITY);
  arm_slider->setVelocity(0.0);
  
  motor_f_r->setPosition(INFINITY);
  motor_f_r->setVelocity(0.0);
  motor_b_l->setPosition(INFINITY);
  motor_b_l->setVelocity(0.0);
  motor_f_l->setPosition(INFINITY);
  motor_f_l->setVelocity(0.0);
  motor_b_r->setPosition(INFINITY);
  motor_b_r->setVelocity(0.0);
  
  motor_f_r_t->setPosition(INFINITY);
  motor_f_r_t->setVelocity(0.0);
  motor_b_l_t->setPosition(INFINITY);
  motor_b_l_t->setVelocity(0.0);
  motor_f_l_t->setPosition(INFINITY);
  motor_f_l_t->setVelocity(0.0);
  motor_b_r_t->setPosition(INFINITY);
  motor_b_r_t->setVelocity(0.0);
  
  int k = 23;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    if (k < 45) {
      arm_slider->setVelocity(slider_speed);
      r_gear_motor->setVelocity(gripper_speed);
      l_gear_motor->setVelocity(gripper_speed);
      arm_motor->setVelocity(arm_speed);
    } else {
      arm_slider->setVelocity(-slider_speed);
      r_gear_motor->setVelocity(-gripper_speed);
      l_gear_motor->setVelocity(-gripper_speed);
      arm_motor->setVelocity(-arm_speed);
      if (k == 90) {k = 0;}
    }
    k = k + 1;
    // Process sensor data here.
    motor_f_r->setVelocity(wheel_speed);
    motor_b_l->setVelocity(wheel_speed);
    motor_f_l->setVelocity(wheel_speed);
    motor_b_r->setVelocity(wheel_speed);
    
    motor_f_r_t->setVelocity(wheel_speed);
    motor_b_l_t->setVelocity(wheel_speed);
    motor_f_l_t->setVelocity(wheel_speed);
    motor_b_r_t->setVelocity(wheel_speed);
    
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
