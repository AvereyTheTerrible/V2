#include "main.h"
//boolean that will be used to track which color to sort out.
//When true, we sort out blue rings and vice versa
//initial state is set by which side's auton runs. 

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define MOGO_OFFSET 1_in


bool clampState = false;

// These are out of 127
const int DRIVE_SPEED = 300;
const int TURN_SPEED = 100;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
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
  chassis.pid_turn_constants_set(3.5, 0.07, 22, 15);
  chassis.pid_swing_constants_set(10.5, 0.5, 150);

}

void blue_sawp() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  // isRed = false;
  int multiplier = -1;

  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED/4);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(1_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(24_in, DRIVE_SPEED / 2, true);//appraoching ring 1
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED/3);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  mogo_constants();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED / 1.5);//backup to prevent cross and align bot
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 2);//getting ring
  chassis.pid_wait_quick_chain();
  pros::delay(180);//giving time to score
  chassis.pid_turn_set(101_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70_in, DRIVE_SPEED,true);//sigma sigma boy sigma boy sigma boy (crossing field)
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  default_constants();
  pros::delay(150);
  chassis.pid_turn_set(2_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED/4);//getting that shit
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  sawp_empty_mogo_constants();
  pros::delay(100);//tune to see how low this can go without sacrificng consistency
  chassis.pid_turn_set(-80_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED);//getting ring 4
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED);//turn to pole
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);//throw that john in reverse
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11_in, DRIVE_SPEED/3);//throw that john in reverse
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
  // isRed = true;
  int multiplier = 1;

  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED/4);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(1_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(24_in, DRIVE_SPEED / 3, true);//appraoching ring 1
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED/3);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  mogo_constants();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED / 1.5);//backup to prevent cross and align bot
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 2);//getting ring
  chassis.pid_wait_quick_chain();
  pros::delay(180);//giving time to score
  chassis.pid_turn_set(101_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-70_in, DRIVE_SPEED,true);//sigma sigma boy sigma boy sigma boy (crossing field)
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  default_constants();
  pros::delay(150);
  chassis.pid_turn_set(2_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-16_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED/4);//getting that shit
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  sawp_empty_mogo_constants();
  pros::delay(100);//tune to see how low this can go without sacrificng consistency
  chassis.pid_turn_set(-80_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED);//getting ring 4
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED/6);//turn to pole
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, DRIVE_SPEED/2);//throw that john in reverse
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11_in, DRIVE_SPEED/3);//throw that john in reverse
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
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  int multiplier = -1;
  // isRed = false;

  chassis.pid_drive_set(-17_in, DRIVE_SPEED/1.125, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED/4);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(1_in, DRIVE_SPEED / 1.5);//reverting the 1 inch excess ahfwofefeo fein
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(23_in, DRIVE_SPEED / 2, true);//appraoching ring 1
  pros::delay(100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg * multiplier, TURN_SPEED/1.5);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED/5);//fast approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  mogo_constants();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED / 1.5);//backup to prevent cross and align bot
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED / 2.2);//getting ring
  pros::delay(100);//giving time to score
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//getting ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED/1.5);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -4_deg * multiplier, SWING_SPEED, 25);//approach to stack 
  chassis.pid_wait_quick_chain();
  //INSERT MODIFICATION FOR STACK INTAKE HERE --------------------------------------------------------
  pros::delay(10000);//TEMP ----------------------
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//swing to angle for break
  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperCylinder.get_value());//doinker deplyoment

}
  

void red_six_ring() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  int multiplier = -1;
  // isRed = true;

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED/1.75);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  empty_mogo_constants();
  chassis.pid_drive_set(2_in, DRIVE_SPEED/3, true);//correcting by 2 inches excess
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(27.5_in, DRIVE_SPEED, true);//appraoching ring 1
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  pros::delay(150);
  chassis.pid_turn_set(120_deg * multiplier, TURN_SPEED);//angling around
  pros::delay(300);
  chassis.pid_swing_set(LEFT_SWING, 85_deg * multiplier, SWING_SPEED/1.5, 30);//getting ring two
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -20_deg * multiplier, SWING_SPEED/1.5, 0);//anlging to ring three
  pros::delay(30);
  intakeMotors.move_velocity(0);//avioding miss-scorin
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -50_deg * multiplier, SWING_SPEED/1.5, 20);//getting ring three
  pros::delay(75);
  intakeMotors.move_velocity(600);//turning intake back on
  chassis.pid_wait_quick_chain();
  mogo_constants();
  chassis.pid_drive_set(2_in, DRIVE_SPEED);//failsafe to grabe ring 3
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2_in, DRIVE_SPEED);//returning to correct pos
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -5_deg * multiplier, SWING_SPEED, 15);//turning away from blue stinky ring
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true);//crossing
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED);//breaking blue
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-5_deg * multiplier, TURN_SPEED);//reset
  sweeperCylinder.set_value(!sweeperCylinder.get_value());
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(23_in, DRIVE_SPEED, true);//continuing cross
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 90_deg * multiplier, SWING_SPEED, -60);//further spin
  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperCylinder.get_value());
  pros::delay(50);
  chassis.pid_swing_set(LEFT_SWING, 160_deg * multiplier, SWING_SPEED, -50);//swing away
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 90_deg * multiplier, SWING_SPEED, 10);//reset with wall
  chassis.pid_wait_quick_chain();
  intakeMotors.move_velocity(600);//turning intake back on
  chassis.pid_drive_set(23, DRIVE_SPEED, true);//getting ring four
  chassis.pid_wait_quick_chain();
  mogo_constants();
  chassis.pid_swing_set(RIGHT_SWING, 0_deg * multiplier, SWING_SPEED, 35);//swinging back to face ring 5(final)
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED);//angling back to stack
  chassis.pid_wait_quick_chain();
  intakeCylinder.set_value(!intakeCylinder.get_value()); //raising intake
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);//aproaching the stack/final ring
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5.5_in, DRIVE_SPEED/3, true);//aproaching the stack/final ring
  pros::delay(50);
  intakeCylinder.set_value(!intakeCylinder.get_value()); //lowering intak
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1_in, DRIVE_SPEED/2, true);//pulling away so as to not intake the wrong ring
  chassis.pid_wait_quick_chain();
  pros::delay(2000);
}


///
// Turn Example
///
void red_mogo_disrupt() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  int multiplier = 1;
  // isRed = true;

  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);//approaching 3rd mogo
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -70_deg * multiplier, SWING_SPEED, 30);//doinking wall
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-110_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-28.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED/4);
  chassis.pid_wait();
  empty_mogo_constants();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-88_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in, DRIVE_SPEED/1.75);
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED/2.25);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  intakeCylinder.set_value(!intakeCylinder.get_value()); //raising intak
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(32.5_deg * multiplier, TURN_SPEED);//angling to rings
  pros::delay(600);
  chassis.pid_drive_set(19_in, DRIVE_SPEED/3);//going to ring
  chassis.pid_wait_quick_chain();
  intakeCylinder.set_value(!intakeCylinder.get_value()); //dropping intak
  pros::delay(500);
  chassis.pid_turn_set(-65_deg * multiplier, TURN_SPEED/1.5);//angling to rings
  chassis.pid_wait_quick_chain();
  pros::delay(5000);//continuing code for as long as possible
}

void blue_mogo_disrupt() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at
  int multiplier = -1;
  // isRed = false;

  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);//approaching 3rd mogo
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -70_deg * multiplier, SWING_SPEED, 30);//doinking wall
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-110_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-29_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED/4);
  chassis.pid_wait();
  empty_mogo_constants();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_drive_set(1.5_in, DRIVE_SPEED/2);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-88_deg * multiplier, TURN_SPEED);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in, DRIVE_SPEED/1.75);
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED/2.25);//angling to rings
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  intakeCylinder.set_value(!intakeCylinder.get_value()); //raising intak
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(32.5_deg * multiplier, TURN_SPEED);//angling to rings
  pros::delay(600);
  chassis.pid_drive_set(19_in, DRIVE_SPEED/3);//going to ring
  chassis.pid_wait_quick_chain();
  intakeCylinder.set_value(!intakeCylinder.get_value()); //dropping intak
  pros::delay(500);
  chassis.pid_turn_set(-65_deg * multiplier, TURN_SPEED/1.5);//angling to rings
  chassis.pid_wait_quick_chain();
  pros::delay(5000);//continuing code for as long as possible
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