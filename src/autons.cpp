#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define MOGO_OFFSET 1_in

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
  chassis.pid_turn_constants_set(2.5, 0.05, 22, 15);
  chassis.pid_swing_constants_set(9, 0.5, 150);

  chassis.pid_turn_exit_condition_set(40_ms, 5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(35_ms, 3.5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(40_ms, 2_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3.5_deg);
  chassis.pid_swing_chain_constant_set(7_deg);
  chassis.pid_drive_chain_constant_set(3.5_in);

  chassis.slew_drive_constants_set(4_in, 40);
}

void mogo_constants() {
  chassis.pid_heading_constants_set(7.5, 2, 50);
  chassis.pid_drive_constants_set(8, 1.4, 23.0767);
  chassis.pid_turn_constants_set(2.8, 0.05, 22, 15); 
  chassis.pid_swing_constants_set(10, 0.8, 100);

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
  chassis.pid_swing_set(LEFT_SWING, 100_deg * multiplier, SWING_SPEED/1.1, 45);//backing
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
  mogo_constants();
  chassis.pid_swing_set(LEFT_SWING, 90_deg * multiplier, SWING_SPEED, 49);//wide curve to align for crossing field
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-42_in, DRIVE_SPEED, true);//crossing through
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, 180_deg * multiplier, SWING_SPEED, 33);//curve to drop mogo and angle towards mogo 2
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25.5_in, DRIVE_SPEED), true;//sprinting towards mogo 2
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED/3);//slowing approach
  chassis.pid_wait_quick_chain();
  clampCylinder.set_value(!clampCylinder.get_value());
  chassis.pid_turn_set(-83.5_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(20_ms, 3_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_drive_set(32_in, DRIVE_SPEED), true;
  chassis.pid_wait_quick_chain();
  intake.move_velocity(0);// turn off to stop ring from getting jiggy wit it
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  intake.move_velocity(600);// turn on to score
  chassis.pid_drive_set(45_in, DRIVE_SPEED), true;
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, DRIVE_SPEED/4), true;
  chassis.pid_wait_quick_chain();
  
}

void six_ring() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  int multiplier = 1;

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED/1.75);//slow approach to mogo with 2 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampCylinder.get_value());
  pros::delay(100);//tune to see how low this can go without sacrificng consistency 
  chassis.pid_drive_set(2_in, DRIVE_SPEED/3, true);//correcting by 2 inches excess
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(140_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(27.5_in, DRIVE_SPEED, true);//appraoching ring 1
  intake.move_velocity(600);//preload scored
  chassis.pid_wait_quick_chain();
  pros::delay(150);
  chassis.pid_turn_set(120_deg * multiplier, TURN_SPEED);//angling around
  pros::delay(300);
  chassis.pid_swing_set(RIGHT_SWING, 85_deg * multiplier, SWING_SPEED/1.5, 30);//getting ring two
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -20_deg * multiplier, SWING_SPEED/1.5, 0);//anlging to ring three
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -50_deg * multiplier, SWING_SPEED/1.5, 20);//getting ring three
  chassis.pid_wait_quick_chain();
  mogo_constants();
  chassis.pid_drive_set(1_in, DRIVE_SPEED);//failsafe to grabe ring 3
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1_in, DRIVE_SPEED);//returning to correct pos
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(RIGHT_SWING, -5_deg * multiplier, SWING_SPEED, 15);//turning away from blue stinky ring
  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperCylinder.get_value());
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);//crossing
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 65_deg * multiplier, SWING_SPEED, 12.5);//swinging into stack
  pros::delay(50);
  chassis.pid_swing_set(RIGHT_SWING, 90_deg * multiplier, SWING_SPEED, -60);//further spin
  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperCylinder.get_value());
  pros::delay(50);
  chassis.pid_swing_set(RIGHT_SWING, 160_deg * multiplier, SWING_SPEED, -50);//swing away
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, 90_deg * multiplier, SWING_SPEED, 10);//reset with wall
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(23, DRIVE_SPEED, true);//getting wall
  chassis.pid_wait_quick_chain();
  mogo_constants();
  chassis.pid_swing_set(LEFT_SWING, 0_deg * multiplier, SWING_SPEED, 35);//swinging back to face ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED);//angling back to stack
  chassis.pid_wait_quick_chain();
  intakeCylinder.set_value(!intakeCylinder.get_value()); //raising intake
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);//aproaching the stack/final ring
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5.5_in, DRIVE_SPEED/3, true);//aproaching the stack/final ring
  intakeCylinder.set_value(!intakeCylinder.get_value()); //lowering intak
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3.5_in, DRIVE_SPEED, true);//pulling away so as to not intake the wrong ring
  chassis.pid_wait_quick_chain();
  pros::delay(2000);
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