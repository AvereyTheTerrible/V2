#include "main.h"
//boolean that will be used to track which color to sort out.
//When true, we sort out blue rings and vice versa
//initial state is set by which side's auton runs. 

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define MOGO_OFFSET 1_in

std::shared_ptr<AsyncPositionController<double, double>> armControlCopy =
AsyncPosControllerBuilder().withMotor({11, -13}).build(); //schmobedying up smth vicious
bool clampState = false;
bool sweeperState = false;

// These are out of 127
const int DRIVE_SPEED = 300;
const int TURN_SPEED = 100;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  armMotor.set_brake_mode_all(MOTOR_BRAKE_HOLD);//this is important

  chassis.pid_heading_constants_set(5.5, 1, 50);
  chassis.pid_drive_constants_set(6.5, 0, 20);
  chassis.pid_turn_constants_set(3, 0.05, 22, 15);
  chassis.pid_swing_constants_set(9, 0.5, 150);

  chassis.pid_turn_exit_condition_set(40_ms, 5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(35_ms, 3.5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(40_ms, 2_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3.5_deg);
  chassis.pid_swing_chain_constant_set(7_deg);
  chassis.pid_drive_chain_constant_set(4.5_in);

  chassis.slew_drive_constants_set(4_in, 40);
}

void mogo_constants() {
  chassis.pid_heading_constants_set(7.5, 1, 50);
  chassis.pid_drive_constants_set(8, 1.4, 23.0767);
  chassis.pid_turn_constants_set(2.8, 0.05, 22, 15); 
  chassis.pid_swing_constants_set(10, 0.5, 100);

}

void empty_mogo_constants() {
  chassis.pid_heading_constants_set(6.5, 1, 50);
  chassis.pid_drive_constants_set(6.5, 0.5, 0);
  chassis.pid_turn_constants_set(2.5, 0.07, 22, 15);
  chassis.pid_swing_constants_set(9, 0.5, 150);

}

void sawp_empty_mogo_constants() {
  chassis.pid_heading_constants_set(6.5, 1, 50);
  chassis.pid_drive_constants_set(7, 0.5, 0);
  chassis.pid_turn_constants_set(4, 0.07, 22, 15);
  chassis.pid_swing_constants_set(10.5, 0.5, 150);

}

void red_FREEZE_IVE_SEEN_THESE_PATHS_BEFORE(){
  int multiplier = 1;

  chassis.pid_heading_constants_set(0, 0, 0);
  armControlCopy->setTarget(1100);//score
  pros::delay(1000);//giving time to score
  chassis.pid_drive_set(-14_in, DRIVE_SPEED/1.125);
  armControlCopy->setTarget(0);//reset arm as we drive
  chassis.pid_wait_quick_chain();
  pros::delay(5000);//TEMP
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_heading_constants_set(5.5, 1, 50);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED/2);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(130);//tune to see how low this can go without sacrificng consistency
  chassis.pid_drive_set(4_in, DRIVE_SPEED);//reverting the 2 inch excess 
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(144_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(24_in, DRIVE_SPEED / 2.5, true);//appraoching ring 1
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED/3.5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(280);
  mogo_constants();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch)
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(130);//giving time to score
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//avoiding blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  
  

}

void blue_sawp() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  //isRed= true;
  int multiplier = -1;

  //intakeMotors.move_velocity(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  //pros::delay(560);
  //intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/2), true;//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(130);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(2_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(144_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(26_in, DRIVE_SPEED / 2.5, true);//appraoching ring 1 -----1 higher than red for temp
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED/3.5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(280);
  mogo_constants();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch)
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(130);//giving time to score
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//avoiding blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(104_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70_in, DRIVE_SPEED,true);//sigma sigma boy sigma boy sigma boy (crossing field)
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(145_deg * multiplier, TURN_SPEED);
  pros::delay(150);
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  default_constants();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(15_deg * multiplier, TURN_SPEED/2.5);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/4);//getting that shit
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  sawp_empty_mogo_constants();
  pros::delay(210);//tune to see how low this can go without sacrificng consistency
  chassis.pid_turn_set(-78_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED);//getting ring 4
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(103_deg * multiplier, TURN_SPEED/3.3, true);//turn to pole
  pros::delay(400);
  chassis.pid_drive_set(42_in, DRIVE_SPEED/2.6);//throw that john in reverse
  chassis.pid_wait_quick_chain();
}
///
// Drive Example
/// standard sawp is red(multiplier is 1)
void red_sawp() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  //isRed= true;
  int multiplier = 1;

  //intakeMotors.move_velocity(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  //pros::delay(560);
  //intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/2), true;//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(130);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(2_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(144_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(24_in, DRIVE_SPEED / 2.5, true);//appraoching ring 1
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED/3.5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(280);
  mogo_constants();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch)
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(130);//giving time to score
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//avoiding blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(104_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70_in, DRIVE_SPEED,true);//sigma sigma boy sigma boy sigma boy (crossing field)
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(145_deg * multiplier, TURN_SPEED);
  pros::delay(150);
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  default_constants();
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(15_deg * multiplier, TURN_SPEED/2.5);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/4);//getting that shit
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  sawp_empty_mogo_constants();
  pros::delay(210);//tune to see how low this can go without sacrificng consistency
  chassis.pid_turn_set(-78_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED);//getting ring 4
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(103_deg * multiplier, TURN_SPEED/3.3, true);//turn to pole
  pros::delay(400);
  chassis.pid_drive_set(39_in, DRIVE_SPEED/2.6);//throw that john in reverse
  chassis.pid_wait_quick_chain();
}

void red_sawp_minimized() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  // isRed = true;
  int multiplier = 1;

  chassis.pid_drive_set(-18_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED/4);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(21_in, DRIVE_SPEED, true);//appraoching ring 1
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(3_in, DRIVE_SPEED / 2.05);//slowing to prevent crossover while grabbing 1
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 100_deg * multiplier, SWING_SPEED/1.1, 45);//backing
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 40_deg * multiplier, SWING_SPEED / 1.5, 20);//1st curve segment towards ring 2
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 190_deg * multiplier, SWING_SPEED /1.5, 10);//2nd curve segment grabbing ring 2 and adjusing towards 3
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 172_deg * multiplier, SWING_SPEED/2.25, 13.5);//3rd curve segment grabbing ring 3
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2_in, DRIVE_SPEED);//ensuring I grab
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//reversing
  pros::delay(40);
  intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  mogo_constants();
  chassis.pid_swing_set(LEFT_SWING, 90_deg * multiplier, SWING_SPEED, 49);//wide curve to align for crossing field
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true);//crossing through
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 180_deg * multiplier, SWING_SPEED, 24);//curve to drop mogo and angle towards mogo 2
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22.5_in, DRIVE_SPEED);//sprinting towards mogo 2
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED/3);//slowing approach
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);
  chassis.pid_turn_set(70_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intakeMotors.move_velocity(600);
  chassis.pid_drive_set(14_in, DRIVE_SPEED/1.5);//slowing approach
  chassis.pid_wait_quick_chain();
  pros::delay(2000);
}



void blue_six_ring() {
    int multiplier = -1;

  //intakeMotors.move_velocity(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  //pros::delay(560);
  //intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/2), true;//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(130);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(2_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(144_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(26_in, DRIVE_SPEED / 2.5, true);//appraoching ring ---- extra one inch than redy
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED/3.5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(280);
  mogo_constants();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch)
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(130);//giving time to score
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);//avoide blue ring 
  chassis.pid_wait_quick_chain();
  
  //sawp divergence point --------

  chassis.pid_turn_set(40_deg * multiplier, TURN_SPEED/1.5);//prepping allingment
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(38_in, DRIVE_SPEED / 3.5);//aapproach to stack
  chassis.pid_wait_quick_chain();
  //chassis.pid_turn_set(-20_deg * multiplier, TURN_SPEED);//prepping allingment
  //chassis.pid_wait_quick_chain();
  //chassis.pid_swing_set(RIGHT_SWING, 0_deg * multiplier, SWING_SPEED, -35);//swing to angle for break
  //chassis.pid_wait_quick_chain();
  //sweeperCylinder.set_value(!sweeperState);
  //sweeperState = !sweeperState;

}
  

void red_six_ring() {
  int multiplier = 1;

  //intakeMotors.move_velocity(-40);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  //pros::delay(560);
  //intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-9_in, DRIVE_SPEED/2), true;//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(130);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(2_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(144_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(26_in, DRIVE_SPEED / 2.5, true);//appraoching ring 1
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED/3.5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(280);
  mogo_constants();
  chassis.pid_drive_set(-11_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch)
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(130);//giving time to score
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);//avoid blue ring
  chassis.pid_wait_quick_chain();
  

  //sawp divergence point --------

  chassis.pid_turn_set(40_deg * multiplier, TURN_SPEED/1.5);//prepping allingment
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(38_in, DRIVE_SPEED / 3.5);//aapproach to stack
  chassis.pid_wait_quick_chain();
  //chassis.pid_turn_set(-20_deg * multiplier, TURN_SPEED);//prepping allingment
  //chassis.pid_wait_quick_chain();
  //chassis.pid_swing_set(RIGHT_SWING, 0_deg * multiplier, SWING_SPEED, -35);//swing to angle for break
  //chassis.pid_wait_quick_chain();
  //sweeperCylinder.set_value(!sweeperState);
  //sweeperState = !sweeperState;
  
}


///
// Turn Example
///
void red_mogo_disrupt() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  //LOOK AT BELOW COMMENT --------------------------------------------------------------!!!!!!!!!!!!!!!!!!!
  int multiplier = 1;//usually red gets neg multiplier and blue pos, but in this path, it is reversed. Sorry
  // isRed = false;
  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED, true);//approaching 3rd mogo---- was 19.5
  pros::delay(1000);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -70_deg * multiplier, SWING_SPEED, 60);//doinking mogo hopefully hard enoguht to mess up other paths
  chassis.pid_wait_quick_chain();
   chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED);//angling to avoid rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
   chassis.pid_turn_set(-118 * multiplier, TURN_SPEED);//angling to clamp
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6.5_in, DRIVE_SPEED/4);
  chassis.pid_wait();
  empty_mogo_constants();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(350);
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-72_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED/1.75);
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED/1.75);//avoiding intake of blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-180_deg * multiplier, TURN_SPEED/2);//turn to face mogo into wall 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);//safeguardinh mohgo
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);
  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);//rushing 3 mohgo
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20_deg * multiplier, TURN_SPEED/2);//turn to face mogo
  chassis.pid_wait_quick_chain();

}

void blue_mogo_disrupt() {
      // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  //LOOK AT BELOW COMMENT --------------------------------------------------------------!!!!!!!!!!!!!!!!!!!
  int multiplier = -1;//usually red gets neg multiplier and blue pos, but in this path, it is reversed. Sorry!!!!
  // isRed = false;
  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED, true);//approaching 3rd mogo --- was 19.5 before
  pros::delay(1000);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -70_deg * multiplier, SWING_SPEED, 60);//doinking mogo hopefully hard enoguht to mess up other paths
  chassis.pid_wait_quick_chain();
   chassis.pid_drive_set(3_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED);//angling to avoid rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
   chassis.pid_turn_set(-118 * multiplier, TURN_SPEED);//angling to clamp
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6.5_in, DRIVE_SPEED/4);
  chassis.pid_wait();
  empty_mogo_constants();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(350);
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-72_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED/1.75);
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_drive_set(-10_in, DRIVE_SPEED/1.75);//avoiding intake of blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-180_deg * multiplier, TURN_SPEED/2);//turn to face mogo into wall 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);//safeguardinh mohgo
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);
  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);//rushing 3 mohgo
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(20_deg * multiplier, TURN_SPEED/2);//turn to face mogo
  chassis.pid_wait_quick_chain();
}

void singlePointSkill(){
  int multiplier = 1;

  //intakeMotors.move_velocity(-40);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  //pros::delay(560);
  //intakeMotors.move_(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED/2), true;//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(200);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(7_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90 * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  pros::delay(400);//temp
  chassis.pid_drive_set(48_in, DRIVE_SPEED / 4, true);//appraoching rings
  pros::delay(130);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED / 2.5, true);//appraoching rings
}
///
// Combining Turn + Drive


///
// Wait Until and Changing Max Speed
///
void third_mogo_elim_red() {
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -45_deg, SWING_SPEED, 0);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  intakeMotors.move_velocity(600);
}
///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

// . . .
// Make your own autonomous functions here!
// . . .