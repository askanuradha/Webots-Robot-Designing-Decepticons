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

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <string>

using namespace webots;

  // FUNCTION DEFINITIONS

  void forward(Robot *robot, double speed) {
    Motor *motor_f_r = robot->getMotor("motorfr");
    Motor *motor_b_l = robot->getMotor("motorbl");
    Motor *motor_f_l = robot->getMotor("motorfl");
    Motor *motor_b_r = robot->getMotor("motorbr");
  
    Motor *motor_f_r_t = robot->getMotor("motorfr_t");
    Motor *motor_b_l_t = robot->getMotor("motorbl_t");
    Motor *motor_f_l_t = robot->getMotor("motorfl_t");
    Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
    motor_f_r->setVelocity(speed);
    motor_b_l->setVelocity(speed);
    motor_f_l->setVelocity(speed);
    motor_b_r->setVelocity(speed);
    
    motor_f_r_t->setVelocity(speed);
    motor_b_l_t->setVelocity(speed);
    motor_f_l_t->setVelocity(speed);
    motor_b_r_t->setVelocity(speed);
  }

  void turn_left(Robot *robot, double speed) {
    Motor *motor_f_r = robot->getMotor("motorfr");
    Motor *motor_b_l = robot->getMotor("motorbl");
    Motor *motor_f_l = robot->getMotor("motorfl");
    Motor *motor_b_r = robot->getMotor("motorbr");
  
    Motor *motor_f_r_t = robot->getMotor("motorfr_t");
    Motor *motor_b_l_t = robot->getMotor("motorbl_t");
    Motor *motor_f_l_t = robot->getMotor("motorfl_t");
    Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
    motor_f_r->setVelocity(speed);
    motor_b_l->setVelocity(0);
    motor_f_l->setVelocity(0);
    motor_b_r->setVelocity(speed);
    
    motor_f_r_t->setVelocity(speed);
    motor_b_l_t->setVelocity(0);
    motor_f_l_t->setVelocity(0);
    motor_b_r_t->setVelocity(speed);
  }
  
  void turn_right(Robot *robot, double speed) {
    Motor *motor_f_r = robot->getMotor("motorfr");
    Motor *motor_b_l = robot->getMotor("motorbl");
    Motor *motor_f_l = robot->getMotor("motorfl");
    Motor *motor_b_r = robot->getMotor("motorbr");
  
    Motor *motor_f_r_t = robot->getMotor("motorfr_t");
    Motor *motor_b_l_t = robot->getMotor("motorbl_t");
    Motor *motor_f_l_t = robot->getMotor("motorfl_t");
    Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
    motor_f_r->setVelocity(0);
    motor_b_l->setVelocity(speed);
    motor_f_l->setVelocity(speed);
    motor_b_r->setVelocity(0);
    
    motor_f_r_t->setVelocity(0);
    motor_b_l_t->setVelocity(speed);
    motor_f_l_t->setVelocity(speed);
    motor_b_r_t->setVelocity(0);
  }
  
int main(int argc, char **argv) {

  Robot *robot = new Robot();

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
  
  Motor *arm_slider = robot->getMotor("arm_slider");
  arm_slider->setPosition(INFINITY);
  arm_slider->setVelocity(0.0);
  
  Camera *camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  
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
  
  DistanceSensor *ir[9];
  std::string names[9] = {"ir1","ir2","ir3","ir4","ir5","ir6","ir7","ir8","ir9"};
  double panel[9] = {0,0,0,0,0,0,0,0,0};
  int powers[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
  
  for (int i=0; i<9; i++) {
    ir[i] = robot->getDistanceSensor(names[i]);
    ir[i]->enable(TIME_STEP);
  };
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {

    forward(robot, wheel_speed);

    // CODE STARTS HERE!!!
    
    // reading ir sensors adding 0 and 1 to the panel array
    for (int i=0; i<9; i++) {
      double sensor_val = ir[i]->getValue();
      if (sensor_val > 700) {
        panel[i] = 0;
      } else {
        panel[i] = 1;
      }
    };
    
    // conversion binary array to decimal number
    // MSB - right ir (ir8)
    int panel_val = 0;
    for (int i=0; i<9; i++) {
      panel_val += panel[i]*powers[i];
    };

    // line folowing logic
    if ((panel_val < 16) & (panel_val != 0)) {
      turn_left(robot, wheel_speed);
    } else if ((panel_val > 32) & (panel_val != 511)) {
      turn_right(robot, wheel_speed);
    } else if ((panel_val == 511) || (panel[4] == 1)) {
      forward(robot, wheel_speed);
    };
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

  
