#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define MOGO_OFFSET 1_in

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 100;
const int SWING_SPEED = 127;

///
// Constants
///
void default_constants() {
  chassis.pid_heading_constants_set(5.5, 0, 50);
  chassis.pid_drive_constants_set(6.5, 0, 20);
  chassis.pid_turn_constants_set(2.5, 0.05, 22, 15);
  chassis.pid_swing_constants_set(6, 0, 100);

  chassis.pid_turn_exit_condition_set(40_ms, 5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(40_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(40_ms, 2_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(4_in, 40);
}

///
// Drive Example
/// standard sawp is red(multiplier is 1)
void sawp() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  int multiplier = 1;

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED/3, true);//slow approach to mogo
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  pros::delay(150);
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  intake.move_velocity(600);//preload scored
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);//appraoching ring 1
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(3_in, DRIVE_SPEED / 2.05);//slowing to prevent crossover while grabbing 1
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 100_deg * multiplier, SWING_SPEED, 30);//backing
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);//reversing
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 40_deg * multiplier, SWING_SPEED / 1.5, 20);//1st curve segment towards ring 2
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 190_deg * multiplier, SWING_SPEED /1.5, 10);//2nd curve segment grabbing ring 2 and adjusing towards 3
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 172_deg * multiplier, SWING_SPEED/2.25, 13.5);//3rd curve segment grabbing ring 3
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4.5_in, DRIVE_SPEED);//ensuring I grab
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4.5_in, DRIVE_SPEED);//reversing
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 90_deg * multiplier, SWING_SPEED, 49);//wide curve to align for crossing field
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-42_in, DRIVE_SPEED, true);//crossing through
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 180_deg * multiplier, SWING_SPEED, 33);//curve to drop mogo and angle towards mogo 2
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-32_in, DRIVE_SPEED), true;//reversing
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4.5_in, DRIVE_SPEED/3);//reversing
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  pros::delay(150);
  chassis.pid_turn_set(-80_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(32_in, DRIVE_SPEED), true;
  chassis.pid_wait_quick_chain();
  intake.move_velocity(0);// turn off to stop ring from getting jiggy wit it
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move_velocity(600);// turn on to score
  chassis.pid_drive_set(40_in, DRIVE_SPEED), true;
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, DRIVE_SPEED/2.5), true;
  chassis.pid_wait_quick_chain();
  

  /*Schassis.pid_swing_set(LEFT_SWING, 180_deg, SWING_SPEED, 15);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  chassis.pid_swing_set(LEFT_SWING, 90, 127, 60);//first curve(wide)
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 180_deg, 127, 18);
  
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-30_in, DRIVE_SPEED);
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(285_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_turn_set(260_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-45_in, DRIVE_SPEED / 2, true);
  */
}

void blue_sawp() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(-19_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3.5_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(-140_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  intake.move_velocity(600);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  chassis.pid_swing_set(RIGHT_SWING, -100_deg, SWING_SPEED, 30, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -40_deg, SWING_SPEED / 1.5, 40, true);
  /*chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 180_deg, SWING_SPEED, 10);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(18_in, DRIVE_SPEED / 2);
  chassis.pid_wait();*/

}

///
// Turn Example
///
void red_right_side() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  intake.move_velocity(600);
  pros::delay(300);
  chassis.pid_turn_set(315_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
}

///
// Combining Turn + Drive
///
void red_left_side() {
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(140_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  intake.move_velocity(600);
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(4.5_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  chassis.pid_swing_set(LEFT_SWING, 100_deg, SWING_SPEED, 30);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 40, SWING_SPEED / 1.5, 20);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 120_deg, SWING_SPEED / 1.5, 25);
  chassis.pid_wait_quick_chain();
}

void blue_left_side()
{
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED / 2);
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

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
  clampCylinder.set_value(!clampCylinder.get_value());
  intake.move_velocity(600);
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