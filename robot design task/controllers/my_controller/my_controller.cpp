// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:
#define TIME_STEP 64
#define slider_speed 0.05
#define wheel_speed 3
#define gripper_speed 0.5
#define arm_speed 0.5

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <string>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Gyro.hpp> 

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
  
    void turn_left_sharp(Robot *robot, double speed) {
    Motor *motor_f_r = robot->getMotor("motorfr");
    Motor *motor_b_l = robot->getMotor("motorbl");
    Motor *motor_f_l = robot->getMotor("motorfl");
    Motor *motor_b_r = robot->getMotor("motorbr");
  
    Motor *motor_f_r_t = robot->getMotor("motorfr_t");
    Motor *motor_b_l_t = robot->getMotor("motorbl_t");
    Motor *motor_f_l_t = robot->getMotor("motorfl_t");
    Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
    motor_f_r->setVelocity(speed);
    motor_b_l->setVelocity(-speed);
    motor_f_l->setVelocity(-speed);
    motor_b_r->setVelocity(speed);
    
    motor_f_r_t->setVelocity(speed);
    motor_b_l_t->setVelocity(-speed);
    motor_f_l_t->setVelocity(-speed);
    motor_b_r_t->setVelocity(speed);
  }
  
  void turn_right_sharp(Robot *robot, double speed) {
    Motor *motor_f_r = robot->getMotor("motorfr");
    Motor *motor_b_l = robot->getMotor("motorbl");
    Motor *motor_f_l = robot->getMotor("motorfl");
    Motor *motor_b_r = robot->getMotor("motorbr");
  
    Motor *motor_f_r_t = robot->getMotor("motorfr_t");
    Motor *motor_b_l_t = robot->getMotor("motorbl_t");
    Motor *motor_f_l_t = robot->getMotor("motorfl_t");
    Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
    motor_f_r->setVelocity(-speed);
    motor_b_l->setVelocity(speed);
    motor_f_l->setVelocity(speed);
    motor_b_r->setVelocity(-speed);
    
    motor_f_r_t->setVelocity(-speed);
    motor_b_l_t->setVelocity(speed);
    motor_f_l_t->setVelocity(speed);
    motor_b_r_t->setVelocity(-speed);
  }
 
  
int main(int argc, char **argv) {

  Robot *robot = new Robot();

  // MOTORS OF WHEELS
  Motor *motor_f_r = robot->getMotor("motorfr");
  Motor *motor_b_l = robot->getMotor("motorbl");
  Motor *motor_f_l = robot->getMotor("motorfl");
  Motor *motor_b_r = robot->getMotor("motorbr");
  Motor *motor_f_r_t = robot->getMotor("motorfr_t");
  Motor *motor_b_l_t = robot->getMotor("motorbl_t");
  Motor *motor_f_l_t = robot->getMotor("motorfl_t");
  Motor *motor_b_r_t = robot->getMotor("motorbr_t");
  
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
  
  PositionSensor *wheel_sensor = robot->getPositionSensor("wheel_sensor");
  wheel_sensor->enable(TIME_STEP);
  //MOTORS OF ARM
  Motor *r_gear_motor = robot->getMotor("r_gear_motor");
  r_gear_motor->setPosition(INFINITY);
  r_gear_motor->setVelocity(0.0);
  Motor *l_gear_motor = robot->getMotor("l_gear_motor");
  l_gear_motor->setPosition(INFINITY);
  l_gear_motor->setVelocity(0.0);

  PositionSensor *arm_sensor = robot->getPositionSensor("arm_sensor");
  arm_sensor->enable(TIME_STEP);
  Motor *arm_motor = robot->getMotor("arm_motor");
  arm_motor->setPosition(INFINITY);
  arm_motor->setVelocity(0.0);
  
  Motor *arm_slider = robot->getMotor("arm_slider");
  arm_slider->setPosition(INFINITY);
  arm_slider->setVelocity(0.0);
  
  TouchSensor *touch_sensor_r = robot->getTouchSensor("touch_sensor_r");
  TouchSensor *touch_sensor_l = robot->getTouchSensor("touch_sensor_l");
  touch_sensor_r->enable(TIME_STEP);
  touch_sensor_l->enable(TIME_STEP);
  
  
  // CAMERA, CAMERA POSITION, CAMERA MOTOR
  Camera *camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  camera->recognitionEnable(TIME_STEP);
  
  
  Motor *cam_motor = robot->getMotor("cam_motor");
  PositionSensor *cam_position = robot->getPositionSensor("cam_position");
  cam_position->enable(TIME_STEP);
  cam_motor->setPosition(INFINITY);
  cam_motor->setVelocity(0.0);
  
  // IR SENSOR ARRAY
  DistanceSensor *ir[10];
  std::string names[10] = {"ir1","ir2","ir3","ir4","ir5","ir6","ir7","ir8","ir9","king_ir"};
  double ir_panel[9] = {0,0,0,0,0,0,0,0,0};
  int powers[9] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
  for (int i=0; i<10; i++) {
    ir[i] = robot->getDistanceSensor(names[i]);
    ir[i]->enable(TIME_STEP);
  };
  
  // LIGHT SENSORS for color detection of dotted line
  LightSensor *color_l = robot->getLightSensor("color_l");
  color_l->enable(TIME_STEP);
  LightSensor *color_r = robot->getLightSensor("color_r");
  color_r->enable(TIME_STEP);

  // ULTRASONIC SENSORS
  DistanceSensor *sonar[9];
  std::string sonar_names[9] = {"sonar_s_r","sonar_s_l","sonar_s_r_b","sonar_s_l_b","sonar_f_r","sonar_f_l","sonar_f","king_sonar","king_lower_sonar"};
  double sonar_panel[9] = {0,0,0,0,0,0,0,0,0};
  for (int i=0; i<9; i++) {
    sonar[i] = robot->getDistanceSensor(sonar_names[i]);
    sonar[i]->enable(TIME_STEP);
  };
  Gyro *gyro = robot->getGyro("gyro");
  gyro->enable(TIME_STEP);
  

  //parts of the task
  bool line_following = 1;
  bool wall_following = 0;
  bool chess_arena = 0;
  bool dotted_line_following = 0;
  
  // switches for some stages
  bool red = 0;
  bool blue = 1;
  
  bool arm_positioned = 0;
  bool object_grabbed = 0;
  bool box_picked_and_turned = 0;
  bool turn_finished = 0;
  bool camera_turned = 0;
  bool king_detected = 0;
  bool robot_positioned = 0;
  bool black_square = 0;
  bool detected = 0;
  bool box_dropped = 0;
  bool red_carpet_detected = 0;
  bool initial_positioned = 0;
  bool turn_left_red = 0;
  bool left_side_free = 0;
  bool turn_right_first = 0;
  bool wall_found = 0;
  bool turn_right_2 = 0;
  bool wall_found_2 = 0;
  bool right_side_free = 0;
  bool turn_left_2 = 0;
  bool t_junction = 0;
  bool blue_turn = 0;
  bool line_detected = 0;
  bool turn = 0;
  bool blue_detect = 0;
  
  // variables for loops
  int time = 1;
  int gripper_time = 1;
  double currentAngle = 0.0;
  int red_time = 1;
  double wheel_start_position = 0;
  int reverse_time = 1;
  int cons = 1;
  
  // others
  bool camera2_turned = 0;
  bool turn2_finished = 0;
  bool turn3_finished = 1;
  bool start_pos = 0;
  bool chess_piece_detected = 0;
  int left_90 = 0;
  
  
  // variables for loops
  double currentAngle1 = 0.0;
  int t = 1;
  int k = 1;
  
  // Main loop:
  while (robot->step(TIME_STEP) != -1) {
  
    if (chess_arena) {
      
      if (turn_finished ==  0) {
        double arm_position = arm_sensor->getValue();
        // getting the arm right position when starting it
        if (arm_positioned == 0) {
          if (arm_position > -0.45) {
            arm_motor->setVelocity(-0.1);
            
            double sonar_1 = sonar[0]->getValue();
            double sonar_2 = sonar[1]->getValue();
            double difference = sonar_1 - sonar_2;
            std::cout<<difference<<std::endl;
            if ((difference < 100) & (difference > -100)) {
              if (difference > 5) {
                turn_right(robot, wheel_speed);
              } else if (difference < -5) {
                turn_left(robot, wheel_speed);
              } else {
                forward(robot, wheel_speed);
              }
            }
          } else {
            arm_motor->setVelocity(0);
            arm_positioned = 1;
          };
        }
        
        // getting touch sensor values
        double touch_val_r = touch_sensor_r->getValue();
        double touch_val_l = touch_sensor_l->getValue();
        
        // getting forward ultrasonic value for calculate the distance to the box
        double sonar_val_f = sonar[6]->getValue();
        
        if ((object_grabbed == 0) & (sonar_val_f > 224) & (arm_positioned == 1)) {
          forward(robot, wheel_speed);
        }
        
        // activating the grippers to grab the object
        if  ((sonar_val_f < 225) & (object_grabbed == 0)) {
          forward(robot, 0);
          r_gear_motor->setVelocity(1);
          l_gear_motor->setVelocity(1);
        };
        // lifting the object when touch sensor values detected
        if ((touch_val_r == 1) & (touch_val_l == 1) & (turn_finished == 0)) {
          forward(robot, 0);
          object_grabbed = 1;
          r_gear_motor->setVelocity(0.05);
          l_gear_motor->setVelocity(0.05);
          if (arm_position < 0.5) {
            arm_motor->setVelocity(0.5);
          } else {
            // after box lifted
            arm_motor->setVelocity(0);
            double ir_5_val = ir[5]->getValue();
            std::cout<<ir_5_val<<std::endl;
            if ((ir_5_val > 400) & (box_picked_and_turned == 0) & (sonar_val_f > 350)) {
              forward(robot,wheel_speed);
            } else { // when there is a chess piece infront of the box
              
              if (currentAngle > -24) {
                turn_right_sharp(robot, 1);
                currentAngle = gyro->getValues()[1]*time;
                time += 1;
                box_picked_and_turned = 1; 
              } else {
                turn_finished = 1;
                currentAngle = 0.0;
                time = 1;
                wheel_start_position = wheel_sensor->getValue();
              }
            }
          };
        };
      };
    
      
      if (turn_finished == 1) {
        // rotating the camera by 90 degrees to detect white king
        // after robot turned
        double ir_5_val = ir[5]->getValue();
        //std::cout<<ir_5_val<<std::endl;
        double cam_pos_val = cam_position->getValue();
        if ((object_grabbed == 1) & (cam_pos_val < 1.55)) {
          cam_motor->setVelocity(0.5);
        } else {
          cam_motor->setVelocity(0.0);
          camera_turned = 1;
        }
        
        // sensor reading
        double king_sonar_val = sonar[7]->getValue();
        double sonar_val_f = sonar[6]->getValue();
        double king_lower_sonar_val = sonar[8]->getValue();
        double king_ir_val = ir[9]->getValue();
        double difference_sonar = king_lower_sonar_val-king_sonar_val;
        std::cout<<"ir "<<king_ir_val<<std::endl;
        std::cout<<"ul "<<king_sonar_val<<std::endl;
        
        
        // going forward
        
        if ((800 < ir_5_val) && (king_detected == 0)) {
          std::cout<<"red carpet detected 1"<<std::endl;
          red_carpet_detected = 1;
          forward(robot,0);
        } else if (((sonar_val_f > 300) && (king_sonar_val == 1000)) && ((camera_turned == 1))) {
          forward(robot, wheel_speed);
          arm_motor->setVelocity(0);
        } else if (sonar_val_f < 301) {
          forward(robot, 0);
          //red_carpet_detected = 1; // actually not red carpet detected but there is a chess piece infront of robot
       
        // king detected
        } else if ((king_sonar_val < 1000) && (((-100 < difference_sonar) & (difference_sonar < 0)) || ((0 < difference_sonar) & (difference_sonar < 100)))) {
            std::cout<<king_ir_val<<std::endl;
            
            if ((king_sonar_val < 95) & (king_ir_val < 115)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 220) & (king_ir_val < 290)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 360) & (king_ir_val < 480)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 486) & (king_ir_val < 657)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 610) & (king_ir_val < 830)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 736) & (king_ir_val < 980)) {
              forward(robot, 0);
              king_detected = 1;
            } else if ((king_sonar_val < 856) & (king_ir_val < 1000)) {
              forward(robot, 0);
              king_detected = 1;
            } else {
              forward(robot, wheel_speed);
            }
          
          
        }
        
        
        if (red_carpet_detected == 1) {
          double wheel_sensor_val = wheel_sensor->getValue();
          //if (red_time < 480) {
          //  forward(robot, -wheel_speed);
          //  red_time += 1;
         // } else {
         //   forward(robot,0);
          //}
          if (wheel_sensor_val < wheel_start_position+14) {
            forward(robot, -wheel_speed);
          } else {
            forward(robot,0);
          }
        }
        
        if (king_detected == 1) {
          // detecting square color
          if (detected == 0) {
            if (ir_5_val < 400) {
              black_square = 0;
            } else {
              black_square = 1;
            }
          }
          detected = 1;
          
          // robot going backward for drop the box
          if (black_square == 0) {
            if ((ir_5_val < 450) & (robot_positioned == 0)) {
              forward(robot, -wheel_speed);
            } else {
              robot_positioned = 1;
            }
          } else {
            if ((ir_5_val > 450) & (robot_positioned == 0)) {
              forward(robot, -wheel_speed);
            } else {
              robot_positioned = 1;
            }
          }
          
          //when robot get positioned
          if (robot_positioned == 1) {
            
            forward(robot, 0);
            double arm_position = arm_sensor->getValue();
            if (arm_position > -0.45) {
              arm_motor->setVelocity(-0.1);
            } else { // when box dropped we need to reset the motor speeds and positions
              arm_motor->setVelocity(0);
              if (gripper_time < 5) {
                l_gear_motor->setVelocity(-3);
                r_gear_motor->setVelocity(-3);
                gripper_time += 1;
              } else {
                l_gear_motor->setVelocity(0);
                r_gear_motor->setVelocity(0);
              }
              
              if (reverse_time < 10) {
                forward(robot,-wheel_speed);
                reverse_time += 1;
              } else {
                box_dropped = 1;
                std::cout<<red_carpet_detected<<std::endl;
              }
            };
          }
          
          /*
                   if (start_pos == 1 ) {
            if (left_side_free == 1) { // there is no chess piece
              if (left_90 == 0) {
                if (currentAngle < 47.5) {
                  red_carpet_detected = 0;
                  turn_left_sharp(robot, 1);
                  currentAngle = gyro->getValues()[1]*time;
                  time += 1; 
                } else {
                  //turn_left_red = 1;
                  turn2_finished = 1;
                  //currentAngle = 0.0;
                  time = 1;
              }
            }
            if (turn2_finished) {
            if ((object_grabbed == 1) & (cam_pos_val < 4.65)) {
              forward(robot,0);
              cam_motor->setVelocity(0.5);
            } else {
              cam_motor->setVelocity(0.0);
              
              //get this position say position 1
              //wheel_sensor_val = wheel_sensor->getValue();
              camera2_turned = 1;
              forward(robot,wheel_speed);
            }
          }
          
          if (camera2_turned){
             // sensor reading
            double king_sonar_val = sonar[7]->getValue();
            double sonar_val_f = sonar[6]->getValue();
            double king_lower_sonar_val = sonar[8]->getValue();
            double difference_sonar = king_lower_sonar_val-king_sonar_val;
      
            // going forward
        
            if ((800 < ir_5_val) && (king_detected == 0)) {
              red_carpet_detected = 1;
              forward(robot,0);
            } else if (((sonar_val_f > 300) && (king_sonar_val == 1000)) && ((camera2_turned == 1) && (red_carpet_detected == 0))) {
              forward(robot, wheel_speed);
              arm_motor->setVelocity(0);
            // king detected
            } else if ((king_sonar_val < 1000) && (((-100 < difference_sonar) & (difference_sonar < 0)) || ((0 < difference_sonar) & (difference_sonar < 100)))) {
              forward(robot, 0);
              king_detected = 1;
            } else if (ir_5_val < 800) {
              forward(robot,0);
              chess_piece_detected = 1;
            }
            
            if (chess_piece_detected == 1) {
              if (t1 < 120){
                forward(robot,-wheel_speed);
                t1 += 1;
              } else {
                forward(robot,0);
                turn3_finished = 0;
                //currentAngle = 0;
                left_90 = 1;
              }
              
              //go back to position1
            if (turn3_finished == 0){
            
              if (currentAngle1 < 23.9) {
                  red_carpet_detected = 0;
                  turn_left_sharp(robot, 1);
                  currentAngle1 = gyro->getValues()[1]*time;
                  
                  time += 1; 
              } else {
                 turn3_finished = 1;
                 time = 1;
                 forward(robot, 0);
                 double arm_position = arm_sensor->getValue();
                 if (arm_position > -0.45) {
                 arm_motor->setVelocity(-0.1);
                 } else { // when box dropped we need to reset the motor speeds and positions
                    arm_motor->setVelocity(0);
                    if (gripper_time < 5) {
                      l_gear_motor->setVelocity(-3);
                      r_gear_motor->setVelocity(-3);
                      gripper_time += 1;
                    } else {
                      l_gear_motor->setVelocity(0);
                      r_gear_motor->setVelocity(0);
                    }
                    box_dropped = 1;
                  };
               }
              }
            }
          }
         }
         }*/
            
            if (red_carpet_detected == 1) {
                 forward(robot,0);
              }
        
        
        if (king_detected == 1) {
          // detecting square color
          if (detected == 0) {
            if (ir_5_val < 400) {
              black_square = 0;
            } else {
              black_square = 1;
            }
          }
          detected = 1;
          
          // robot going backward for drop the box
          if (black_square == 0) {
            if ((ir_5_val < 450) & (robot_positioned == 0)) {
              forward(robot, -wheel_speed);
            } else {
              robot_positioned = 1;
              std::cout<<"jdgcjajca"<<std::endl;
            }
          } else {
            if ((ir_5_val > 450) & (robot_positioned == 0)) {
              forward(robot, -wheel_speed);
            } else {
              robot_positioned = 1;
              std::cout<<"asasasasa"<<std::endl;
            }
          }
          
          //when robot get positioned
          if (robot_positioned == 1) {
            
            forward(robot, 0);
            double arm_position = arm_sensor->getValue();
            if (arm_position > -0.45) {
              arm_motor->setVelocity(-0.1);
            } else { // when box dropped we need to reset the motor speeds and positions
              arm_motor->setVelocity(0);
              if (gripper_time < 5) {
                l_gear_motor->setVelocity(-3);
                r_gear_motor->setVelocity(-3);
                gripper_time += 1;
              } else {
                l_gear_motor->setVelocity(0);
                r_gear_motor->setVelocity(0);
              }
              box_dropped = 1;
            };
          }
          
          
          
         }
      }
      
      // after box dropped
      if (box_dropped == 1) {
        // sensor reading
        double king_sonar_val = sonar[7]->getValue();
        double sonar_val_f = sonar[6]->getValue();
        double king_lower_sonar_val = sonar[8]->getValue();
            
        if (red_carpet_detected == 1) { // if red carpet detected then first row does not have the king
          if ((king_lower_sonar_val > 300) & (left_side_free == 0)) {
            left_side_free = 1;
          }
          // Assume robot initial positioned and lower ir turn left relative to the robot
          if (left_side_free == 1) { // there is no chess piece
            if (turn_left_red == 0) {
              if (currentAngle < 23.9) {
                turn_left_sharp(robot, 1);
                currentAngle = gyro->getValues()[1]*time;
                time += 1; 
              } else {
                turn_left_red = 1;
                currentAngle = 0.0;
                time = 1;
              }
            }
            if (turn_left_red == 1) {
              if ((sonar_val_f < 300) & (wall_found == 0)) {
                wall_found = 1;
              }
              if (wall_found == 1) {
                if (turn_right_first == 0) {
                  if (currentAngle > -24) {
                    turn_right_sharp(robot, 1);
                    currentAngle = gyro->getValues()[1]*time;
                    time += 1; 
                  } else {
                    turn_right_first = 1;
                    currentAngle = 0.0;
                    time = 1;
                    std::cout<<"dhvjsd"<<std::endl;
                    forward(robot, wheel_speed);
                  }
                }
                
                if ((sonar_val_f < 300) & (wall_found_2 == 0) & (turn_right_first == 1)) {
                  wall_found_2 = 1;
                }
                if (wall_found_2 == 1) {
                  if (turn_right_2 == 0) {
                    if (currentAngle > -24) {
                      turn_right_sharp(robot, 1);
                      currentAngle = gyro->getValues()[1]*time;
                      time += 1; 
                    } else {
                      turn_right_2 = 1;
                      currentAngle = 0.0;
                      time = 1;
                      forward(robot, wheel_speed);
                    }
                  } else if (turn_right_2 == 1) {
                    forward(robot, wheel_speed);
                  }
                }
              } else {
                forward(robot, wheel_speed);
              }
            }
          } else { // if left side not free then go forward one square
            double wheel_sensor_val = wheel_sensor->getValue();
            if (wheel_sensor_val < wheel_start_position + 12) {
              forward(robot, wheel_speed);
            } else {
              
            }
          }
        } else {
          if (turn_right_first == 0) {
            std::cout<<currentAngle<<std::endl;
            if (currentAngle > -24) {
              turn_right_sharp(robot, 1);
              currentAngle = gyro->getValues()[1]*time;
              time += 1; 
            } else {
              turn_right_first = 1;
              currentAngle = 0.0;
              time = 1;
              
              forward(robot, wheel_speed);
            }
          }
          if (turn_right_first == 1) {
            if ((sonar_val_f < 350) & (turn_right_first == 1)) {
              right_side_free = 1;
            }
            
            if ((right_side_free == 0) & (turn_left_red == 0)) {
              forward(robot, wheel_speed);
            } else {
              if (turn_left_red == 0) {
                if (currentAngle < 24) {
                  turn_left_sharp(robot, 1);
                  currentAngle = gyro->getValues()[1]*time;
                  time += 1; 
                } else {
                  turn_left_red = 1;
                  currentAngle = 0.0;
                  time = 1;
                  forward(robot, wheel_speed);
                }
              }
            }
            if (turn_left_red == 1) {
              forward(robot,wheel_speed);
            }
            
            if ((sonar_val_f < 350) & (turn_left_red == 1)) {
              wall_found = 1;
            }
            if ((wall_found == 1) & (turn_left_2 == 0)) {
              if (turn_left_2 == 0) {
                if (currentAngle < 23.8) {
                  turn_left_sharp(robot, 1);
                  currentAngle = gyro->getValues()[1]*time;
                  time += 1; 
                } else {
                  turn_left_2 = 1;
                  currentAngle = 0.0;
                  time = 1;
                }
              }
            }
            if (turn_left_2 == 1) {
              double sonic_val = sonar[0]->getValue();
              if (sonic_val < 248) {
                turn_left(robot, wheel_speed);
              } else if (sonic_val > 253) {
                turn_right(robot, wheel_speed);
              } else {
                forward(robot,wheel_speed);
              }
            }
          }
          }
        }
      }
    }  
    //arm_slider->setVelocity(0.1);

    // CODE STARTS HERE!!!
    
    if (line_following == 1) {
      // reading ir sensors adding 0 and 1 to the panel array
      for (int i=0; i<9; i++) {
        double sensor_val = ir[i]->getValue();
        if (sensor_val > 700) {
          ir_panel[i] = 0;
        } else {
          ir_panel[i] = 1;
        }
      };
      
      // conversion binary array to decimal number
      // MSB - right ir (ir8)
      int panel_val = 0;
      for (int i=0; i<9; i++) {
        panel_val += ir_panel[i]*powers[i];
      };

      // line folowing algorithm
      if ((panel_val == 511) || (ir_panel[4] == 1)) {
        forward(robot, wheel_speed);
      }
      else if ((panel_val < 16) & (panel_val != 0)) {
        turn_left(robot, wheel_speed);
      } else if ((panel_val > 32) & (panel_val != 511)) {
        turn_right(robot, wheel_speed);
      } else if (panel_val == 0) {
        forward(robot, wheel_speed);
        wall_following = 1;
        line_following = 0;
      } else {
        forward(robot, wheel_speed);
      };
    };

    // wall following part
    if (wall_following == 1) {
    
      for (int i=0; i<9; i++) {
        double sensor_val = ir[i]->getValue();
        if (sensor_val > 700) {
          ir_panel[i] = 0;
        } else {
          ir_panel[i] = 1;
        }
      };
      // get the ultrasonic readings
      
      for (int i=0; i<7; i++) {
        double sonar_val = sonar[i]->getValue();
        sonar_panel[i] = round(sonar_val*100)/100;
      };
      int panel_val = 0;
      for (int i=0; i<9; i++) {
        panel_val += ir_panel[i]*powers[i];
      };
      std::cout<<panel_val<<std::endl;
      
      forward(robot, wheel_speed);
      // wall following algorithm
      if ((sonar_panel[0] > sonar_panel[1]) & (sonar_panel[0] < 1000)) {
        turn_right(robot, wheel_speed);
      } else if ((sonar_panel[0] < sonar_panel[1]) & (sonar_panel[1] < 1000)) {
        turn_left(robot, wheel_speed);
      } else if ((sonar_panel[0] == sonar_panel[1]) & (sonar_panel[0] < 1000) & (sonar_panel[1] < 1000)) {
        forward(robot, wheel_speed);
      } else if ((sonar_panel[0] < 1000) & (sonar_panel[0] > 260)) {
        turn_right(robot, wheel_speed);
      } else if ((sonar_panel[0] < 1000) & (sonar_panel[0] < 260)) {
        turn_left(robot, wheel_speed);
      } else if ((sonar_panel[1] < 1000) & (sonar_panel[1] > 260)) {
        turn_left(robot, wheel_speed);
      } else if ((sonar_panel[1] < 1000) & (sonar_panel[1] < 260)) {
        turn_right(robot, wheel_speed);
      } else {
        forward(robot, wheel_speed);
      };
      
      if (panel_val == 511) {
        
        wall_following = 0;
        dotted_line_following = 1;
      }
    }
    
    // dotted line following
    if (dotted_line_following) {
    std::cout<<"asasslklklk"<<std::endl;
    blue = 0;
    red = 1;
      double color = 0;
      if  (red){
        color = 500;
      }
      else if (blue){
        color = 800;
      }
      // reading ir sensors adding 0 and 1 to the panel array
      for (int i=0; i<9; i++) {
        double sensor_val = ir[i]->getValue();
        if (sensor_val > color) {
          ir_panel[i] = 0;
        } else {
          ir_panel[i] = 1;
        }
      };
      // conversion binary array to decimal number
      // MSB - right ir (ir8)
      int panel_val = 0;
      for (int i=0; i<9; i++) {
        panel_val += ir_panel[i]*powers[i];
      };

      // line folowing algorithm
      if (blue) {
        if (k < 50) {
          forward(robot, wheel_speed);
          k += 1;
        } else {
          turn = 1;
        }
        
        double ir_left = ir[0]->getValue();
        double ir_right = ir[8]->getValue();
        std::cout<<ir_left<<" left"<<std::endl;
        std::cout<<ir_right<<" right"<<std::endl;
        
        if ((ir_left < 600) & (ir_left > 500) & (ir_right > 700) & (ir_right < 1000) & (blue_turn == 0)) {
          blue_detect = 1;
          
        }
        if ((blue_detect == 1) & (blue_turn == 0)) {
          if (cons < 15) {
              turn_right_sharp(robot,1);
              cons += 1;
              std::cout<<cons<<std::endl;
            } else {
              blue_turn = 1;
              forward(robot, 1);
            }
        }
        
        std::cout<<cons<<std::endl;
        if (blue_turn == 1) {
          std::cout<<"run"<<std::endl;
          if (panel_val == 511) {
            forward(robot, wheel_speed);
          } 
           
          else if ((panel_val < 16) & (panel_val != 0)) {
            turn_left_sharp(robot, wheel_speed);
          } else if ((panel_val > 32) & (panel_val != 511)) {
            turn_right_sharp(robot, wheel_speed);
          } else if ((panel_val == 511) || (ir_panel[4] == 1)) {
            forward(robot, wheel_speed);
          } else if (panel_val == 0) {
            forward(robot, wheel_speed);
          } else {
            forward(robot, wheel_speed);
          };
        }
        
      }
      
      if (red) {
        double ir_left = ir[0]->getValue();
        double ir_right = ir[8]->getValue();
        double dif = ir_right- ir_left;
        
        if (t_junction == 0) {
        
          if (blue_turn == 0) {
            if ((ir_left < 600) & (ir_left > 500) & (ir_right > 700) & (ir_right < 1000)) {
              line_detected = 1;
            } else {
              forward(robot, wheel_speed);
            }
            if ((cons < 45) & (line_detected == 1)) {
              turn_left_sharp(robot,wheel_speed);
              cons += 1;
            } else {
              blue_turn = 1;
              forward(robot, wheel_speed);
            }
            
          }
          
          
          if ((panel_val == 511) || (ir_panel[4] == 1)) {
            forward(robot, wheel_speed);
          }
          else if ((panel_val > 32) & (panel_val != 511)) {
            turn_right_sharp(robot, wheel_speed);
          } else if ((panel_val < 16) & (panel_val != 0)) {
            turn_left_sharp(robot, wheel_speed);
          } else if (panel_val == 0) {
            forward(robot, wheel_speed);
          } else {
            forward(robot, wheel_speed);
          };
        }
        if (cons < 45) {
          if ((ir_left < 420) & (ir_right > 900)) {
            turn_left_sharp(robot, 1);
            cons += 1;
            t_junction = 1;
            std::cout<<cons<<std::endl;
          }
          if (cons > 1) {
            cons = cons + 1;
          }
          
        } else {
              // line folowing algorithm
          if ((panel_val < 16) & (panel_val != 0)) {
            turn_left(robot, wheel_speed);
          } else if ((panel_val > 32) & (panel_val != 511)) {
            turn_right(robot, wheel_speed);
          } else if ((panel_val == 511) || (ir_panel[4] == 1)) {
            forward(robot, wheel_speed);
          } else if (panel_val == 0) {
            forward(robot, wheel_speed);
            line_following = 0;
          } else {
            forward(robot, wheel_speed);
          };
        }
      }
      
      double ir_3_val = ir[3]->getValue();
      double ir_4_val = ir[4]->getValue();
      double ir_5_val = ir[5]->getValue();
      double ir_6_val = ir[6]->getValue();
      double ir_7_val = ir[7]->getValue();
      double ir_8_val = ir[7]->getValue();
      if ((ir_3_val < 700) & (ir_3_val > 600) & (ir_4_val < 700) & (ir_4_val > 600) & (ir_5_val < 700) & (ir_5_val > 600) & (ir_6_val < 700) & (ir_6_val > 600) & (ir_7_val < 700) & (ir_7_val > 600)) {
        dotted_line_following = 0;
        chess_arena = 1;
      }
      if ((blue) & ((ir_8_val > 600) & (ir_8_val < 980)) & (blue_turn == 1)) {
        dotted_line_following = 0;
        chess_arena = 1;
        forward(robot, wheel_speed);
        std::cout<<"skdbvks"<<std::endl;
      }
    }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

  
