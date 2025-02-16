#include "autons.hpp"
#include "main.h"
#include "okapi/api/device/motor/abstractMotor.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
//boolean that will be used to track which color to sort out.
//When true, we sort out blue rings and vice versa
//initial state is set by which side's auton runs.

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

#define MOGO_OFFSET 1_in

auto armControlCopy = AsyncPosControllerBuilder()
                                          .withMotor({18, -11})
                                          .withGearset(okapi::AbstractMotor::GearsetRatioPair(okapi::AbstractMotor::gearset::green, 48.0 / 12.0))
																					.build(); //schmobedying up smth vicious
bool clampState = false;
bool sweeperState = false;

double intakeSpeed = 600;
bool isRed = true;

// These are out of 127
const int DRIVE_SPEED = 300;
const int TURN_SPEED = 100;
const int SWING_SPEED = 127;

pros::Optical colorSensor(19);

void colorSort()
{
    //must be conigured to sensor and conditions
    int blueThreshold = 210;
    int redThreshold = 8;
    
    while(true)
    {
        if (colorSensor.get_proximity() > 230)
        {
            if(colorSensor.get_hue() >= blueThreshold)
            {
                std::cout << "blue" << '\n';
                //if the ring is blue and we want red, sort out
                if(isRed)
                {
                    pros::delay(30);
                    intakeSpeed = -600;
                    pros::delay(350);//waiting
                    intakeSpeed = 600;
                } 
            }
            
            else if(colorSensor.get_hue() <= redThreshold)
            {
                std::cout << "red" << '\n';
                //if the ring is red and we want blue, sort out
                if(!isRed)
                {
                    pros::delay(30);
                    intakeSpeed = -600;
                    pros::delay(350);//waiting
                    intakeSpeed = 600;
                } 
            }
    
            pros::delay(10); 
        }
    }
}

///
// Constants
///
void default_constants() {
  armMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);//this is important
  
  chassis.pid_heading_constants_set(4.3, .5, 50);
  chassis.pid_drive_constants_set(12, 0, 20);
  chassis.pid_turn_constants_set(5, 0.05, 21, 15);
  chassis.pid_swing_constants_set(9, 0.5, 150);

  chassis.pid_turn_exit_condition_set(40_ms, 2_deg, 250_ms, 3_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(35_ms, 3.5_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(20_ms, 3_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3.5_deg);
  chassis.pid_swing_chain_constant_set(7_deg);
  chassis.pid_drive_chain_constant_set(4.5_in);
  chassis.slew_drive_set(false);
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
  chassis.pid_heading_constants_set(6.8, 1, 50);
  chassis.pid_drive_constants_set(7.2, 0.7, 0);
  chassis.pid_turn_constants_set(4, 0.07, 22, 15);
  chassis.pid_swing_constants_set(10.5, 0.5, 150);

}

void skills()
{
  chassis.odom_xyt_set(72, 17, 180);

  armControlCopy->setTarget(480);//score
  pros::delay(800);
  armControlCopy->setTarget(0);//reset arm as we drive
  pros::delay(100);
  //  
  chassis.pid_odom_set({{102, 24}, rev, 80});
  chassis.pid_wait();
  pros::delay(100);

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();

  chassis.pid_odom_set({{97, 50}, fwd, 110});
  pros::delay(140);
  intakeMotors.move_velocity(intakeSpeed);
  chassis.pid_wait();
  pros::delay(100);


  chassis.pid_odom_set({{121, 95}, fwd, 110});
  pros::delay(600);
  armControlCopy->setTarget(37.4);//score
  chassis.pid_wait();

  chassis.pid_odom_set({{118, 72}, rev, 110});
  chassis.pid_wait();

  /*
  intakeMotors.move_relative(-10, 600);
  while (intakeMotors.get_position() != intakeMotors.get_target_position())
      pros::delay(5);
  */

  intakeMotors.move_velocity(-600);
  pros::delay(170);
  intakeMotors.move_velocity(0);
  pros::delay(500);

  armControlCopy->setMaxVelocity(60);

  armControlCopy->setTarget(150);//score
  pros::delay(300);
  intakeMotors.move_velocity(intakeSpeed);

  chassis.pid_odom_set({{134, 72}, fwd, 35});
  chassis.pid_wait();

  armControlCopy->setMaxVelocity(200);

  // score on wallstake
  armControlCopy->setTarget(310);//score
  pros::delay(700);

  // back up and arm down
  armControlCopy->setTarget(0);
  chassis.pid_odom_set({{123, 71}, rev, 110});
  chassis.pid_wait();

  // turn towards the last 4 rings
  chassis.pid_odom_set({{123, 45}, fwd, 110});
  chassis.pid_wait();

  // pick up 2nd ring
  chassis.pid_odom_set({{123, 25}, fwd, 50});
  chassis.pid_wait();

  // pick up 3rd ring
  chassis.pid_odom_set({{123, 14}, fwd, 20});
  chassis.pid_wait();

  // pick up last ring
  chassis.pid_odom_set({{131, 24}, fwd, 50});
  chassis.pid_wait();

  // put in corner
  chassis.pid_odom_set({{136, 11}, rev, 90});
  chassis.pid_wait();

  intakeMotors.move_velocity(-500);
  pros::delay(160);
  intakeMotors.move_velocity(0);

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();
  
  chassis.pid_drive_constants_set(16, 0, 36); // increase kp for long movements

  chassis.pid_odom_set({{122, 95}, fwd, 127});
  intakeMotors.move_velocity(intakeSpeed);
  armControlCopy->setTarget(37);//score
  chassis.pid_wait();

  /*--------------------------------DO NOT DELETE THE LINE RIGHTE BELOW THIS----------------------------------------------s*/
  default_constants(); // reset pid constants for short movements

  chassis.pid_odom_set({{122, 103}, fwd, 70});
  chassis.pid_wait();

  // get 2nd mogo
  chassis.pid_odom_set({{97, 123}, rev, 60});
  chassis.pid_wait();

  intakeMotors.move_velocity(0);

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();

  sweeperCylinder.set_value(!sweeperState);
  sweeperState = !sweeperState;

  chassis.pid_odom_set({{120, 122}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set({{120, 115}, fwd, 110});
  chassis.pid_wait();

  sweeperCylinder.set_value(!sweeperState);
  sweeperState = !sweeperState;

  chassis.pid_odom_set({{130, 122}, rev, 70});
  chassis.pid_wait();

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();

  intakeMotors.move_velocity(-600);
  pros::delay(200);
  intakeMotors.move_velocity(0);

  armControlCopy->setMaxVelocity(65);

  armControlCopy->setTarget(150);//score
  pros::delay(300);

  chassis.pid_odom_set({{115, 105.5}, fwd, 110});
  chassis.pid_wait();

  chassis.pid_odom_set({{75.5, 105.5}, rev, 60});
  chassis.pid_wait();

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(200);
  chassis.pid_wait();

  chassis.pid_odom_set({{75.5, 119}, fwd, 60});
  chassis.pid_wait();

  chassis.pid_odom_set({{75.5, 113}, rev, 110});
  chassis.pid_wait();

  armControlCopy->setMaxVelocity(200);

  armControlCopy->setTarget(480);//score
  pros::delay(700);

  intakeMotors.move_velocity(intakeSpeed);

  armControlCopy->setTarget(0);//score
  pros::delay(200);

  // back up

  chassis.pid_odom_set({{76, 100}, rev, 110});
  chassis.pid_wait();

  // pick up first ring

  chassis.pid_odom_set({{102, 79}, fwd, 80});
  chassis.pid_wait();

  // pick up ring in the middle

  chassis.pid_odom_set({{80, 60}, fwd, 110});
  chassis.pid_wait();

  // pick up the first of the 4 

  chassis.pid_odom_set({{64, 43}, fwd, 110});
  chassis.pid_wait();

  // pick up the 2nd one
  chassis.pid_odom_set({{40, 15}, fwd, 70});
  chassis.pid_wait();
  // pick up the 3rd
  
  chassis.pid_odom_set({{38, -5}, fwd, 70});
  chassis.pid_wait();

  // pick up thelast ring

  chassis.pid_odom_set({{18, 8}, fwd, 70});
  chassis.pid_wait();

  // move to corner

  chassis.pid_odom_set({{18, -4}, rev, 70});
  chassis.pid_wait();

  intakeMotors.move_velocity(-500);
  pros::delay(160);
  intakeMotors.move_velocity(0);

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();

  intakeMotors.move_velocity(intakeSpeed);
  armControlCopy->setTarget(37);//score
  chassis.pid_wait();

  // move to ring
  chassis.pid_odom_set({{29, 30}, fwd, 110});
  chassis.pid_wait();

  // move to mogo
  chassis.pid_odom_set({{60, 5}, rev, 60});
  chassis.pid_wait();

  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait();
  intakeMotors.move_velocity(0);

  pros::delay(200);

  chassis.pid_odom_set({{30, 52}, fwd, 110});
  pros::delay(200);

  chassis.pid_wait();

  armControlCopy->setMaxVelocity(60);

  armControlCopy->setTarget(150);//score
  pros::delay(300);
  intakeMotors.move_velocity(intakeSpeed);

  chassis.pid_odom_set({{15, 52}, fwd, 60});
  chassis.pid_wait();

  armControlCopy->setMaxVelocity(200);

  // score on wallstake
  armControlCopy->setTarget(310);//score
  pros::delay(700);

  // back up and arm down
  intakeMotors.move_velocity(intakeSpeed);

  armControlCopy->setTarget(0);
  pros::delay(300);


  chassis.pid_odom_set({{26, 52}, rev, 110});
  chassis.pid_wait();

  // get the ring
  chassis.pid_odom_set({{26, 75}, fwd, 110});
  chassis.pid_wait();

  // get the mext ring
  chassis.pid_odom_set({{53, 85}, fwd, 110});
  chassis.pid_wait();

  // get the mext ring
  chassis.pid_odom_set({{26, 95}, fwd, 110});
  chassis.pid_wait();

   // get the ring after that
   chassis.pid_odom_set({{24, 105}, fwd, 60});
   chassis.pid_wait();

   // back up and get the ring after that
   chassis.pid_odom_set({{26, 98}, rev, 110});
   chassis.pid_wait();

   chassis.pid_odom_set({{5, 98}, fwd, 70});
   chassis.pid_wait();

   // back up and clear the corner
   chassis.pid_odom_set({{10, 75}, rev, 70});
   chassis.pid_wait();

   sweeperCylinder.set_value(!sweeperState);
   sweeperState = !sweeperState;

   // clear corner
   chassis.pid_odom_set({{10, 95}, fwd, 70});
   chassis.pid_wait();

    // place mogo in corner
    chassis.pid_odom_set({{4, 92}, rev, 110});
    chassis.pid_wait();
}

void ringrush_SAWP(){     
  isRed = true;
  int multiplier = 1;

  armMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);//this is important
  chassis.pid_heading_constants_set(0, 0, 0);

  intakeMotors.move_velocity(intakeSpeed);
  chassis.pid_drive_set(43_in, DRIVE_SPEED); // rush the ring stack
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  chassis.pid_wait_quick_chain();

// back up
  chassis.pid_drive_set(-4_in, DRIVE_SPEED);//getting ring 3
  pros::delay(200);
  intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();

  
  chassis.pid_heading_constants_set(5.5, 1, 50);
  // turn to mogo
  chassis.pid_turn_set(-40, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
 
  // move to mogo
  chassis.pid_drive_set(-24, DRIVE_SPEED / 1.25);
  chassis.pid_wait_quick_chain();
  
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(120);
  chassis.pid_drive_set(9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
// move to rings

  intakeMotors.move_velocity(intakeSpeed);
  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  chassis.pid_drive_set(13, DRIVE_SPEED / 10);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_turn_set(128_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(34.5, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(18, DRIVE_SPEED, true);
  pros::delay(200);
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2.5, DRIVE_SPEED/10);
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  
  pros::delay(700);

 // armControlCopy->setTarget(64);//score
  armControlCopy->setTarget(66);//score

  pros::delay(700);

  chassis.pid_turn_set(260, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(170_deg, TURN_SPEED);
  intakeMotors.move_velocity(0);

  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState; 

  


  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
   
  chassis.pid_drive_set(-4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  armControlCopy->setTarget(500);//score
  pros::delay(  1000);

  chassis.pid_drive_set(-23, DRIVE_SPEED / 6, true);

  /*// go for third ring and alliance stake
  chassis.pid_turn_set(115_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(31_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

 chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(13_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  // let go of mogo
  clampCylinder.set_value(!clampState);
  clampState = !clampState;

  chassis.pid_drive_set(34_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

    chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;

  chassis.pid_turn_set(260_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();

  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;

  armControlCopy->setTarget(100);//score


  intakeMotors.move_velocity(0);

  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-6_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  armControlCopy->setTarget(520);//score*/


}

void blueRingRush(){
  isRed = true;
  int multiplier = 1;

  armMotor.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);//this is important
  chassis.pid_heading_constants_set(0, 0, 0);

 intakeMotors.move_velocity(intakeSpeed);
  chassis.pid_drive_set(42_in, DRIVE_SPEED); // rush the ring stack
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  // turn to mogo
  chassis.pid_turn_set(50, TURN_SPEED);
  pros::delay(200);
  intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_heading_constants_set(5.5, 1, 50);
 
   chassis.pid_drive_set(-26, DRIVE_SPEED / 1.25);
  chassis.pid_wait_quick_chain();
  
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(120);
  chassis.pid_drive_set(4_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
// move to rings

intakeMotors.move_velocity(intakeSpeed);
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  chassis.pid_drive_set(13, DRIVE_SPEED / 10);//getting ring 3
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_turn_set(-128_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(34.5, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  
  chassis.pid_drive_set(18, DRIVE_SPEED, true);
  pros::delay(200);
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(2.5, DRIVE_SPEED/10);
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  
  pros::delay(700);

 // armControlCopy->setTarget(64);//score
  armControlCopy->setTarget(66);//score

  pros::delay(700);

  chassis.pid_turn_set(260, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState;
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-170_deg, TURN_SPEED);
  intakeMotors.move_velocity(0);

  chassis.pid_wait_quick_chain();
  sweeperCylinder.set_value(!sweeperState); //deploy doinker
  sweeperState = !sweeperState; 


  chassis.pid_drive_set(20, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
   
  chassis.pid_drive_set(-4, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  armControlCopy->setTarget(500);//score
  pros::delay(  1000);

  chassis.pid_drive_set(-23, DRIVE_SPEED / 6, true);
}

void red_FREEZE_IVE_SEEN_THESE_PATHS_BEFORE(){
  isRed = true;
  int multiplier = 1;
  pros::Task colorSortThread(colorSort);

  chassis.pid_heading_constants_set(0, 0, 0);

  armControlCopy->setTarget(520);//score
  pros::delay(800);
  chassis.pid_drive_set(-11_in, DRIVE_SPEED/1.125);
  armControlCopy->setTarget(0);//reset arm as we drive
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_heading_constants_set(5.5, 1, 50);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED/1.125);//slow approach to mogo with 8 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(100);//tune to see how low this can go without sacrificng consistency
  chassis.pid_drive_set(4_in, DRIVE_SPEED);//reverting 4 inchES excess 
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(142_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(22_in, DRIVE_SPEED);//appraoching ring 1
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85 * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12, DRIVE_SPEED/7);// approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(120);
  mogo_constants();
  //chassis.pid_drive_set(-5_in, DRIVE_SPEED);//backup to prevent align bot(added 1 inch) MARkED
  //chassis.pid_wait_quick_chain();
  //pros::delay(200);

  chassis.pid_swing_set(LEFT_SWING, 0_deg * multiplier, SWING_SPEED, -25);//turning to ring 3 about the right side
  intakeMotors.move_velocity(0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(14_in, DRIVE_SPEED / 1.4);//getting ring 3
  intakeMotors.move_velocity(600);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED);//avoiding blue ring
  chassis.pid_wait_quick_chain();

  // qual version

  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  armControlCopy->setTarget(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, DRIVE_SPEED/3);

  // elim version

  /*chassis.pid_turn_set(25, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();*/

  colorSortThread.join();
}

void blue_FREEZE_IVE_SEEN_THESE_PATHS_BEFORE(){
  isRed = false;
  int multiplier = -1;


  chassis.pid_heading_constants_set(0, 0, 0);

  armControlCopy->setTarget(520);//score
  pros::delay(800);
  chassis.pid_drive_set(-11_in, DRIVE_SPEED/1.125);
  armControlCopy->setTarget(0);//reset arm as we drive
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg * multiplier, TURN_SPEED);
  chassis.pid_heading_constants_set(5.5, 1, 50);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-22_in, DRIVE_SPEED);//approaching at full speed
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED/2);//slow approach to mogo with 8 inches exccess
  chassis.pid_wait();
  clampCylinder.set_value(!clampState);
  clampState = !clampState;
  pros::delay(40);//tune to see how low this can go without sacrificng consistency
  chassis.pid_drive_set(6_in, DRIVE_SPEED);//reverting 4 inchES excess 
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(148_deg * multiplier, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  sawp_empty_mogo_constants();
  intakeMotors.move_velocity(600);//preload scored
  chassis.pid_drive_set(22.5_in, DRIVE_SPEED);//appraoching ring 1
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(86_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(12_in, DRIVE_SPEED/10);// approach to ring 2
  chassis.pid_wait_quick_chain();
  pros::delay(120);
  mogo_constants();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED / 2.5);//backup to prevent align bot(added 1 inch) MARkED
  pros::delay(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(LEFT_SWING, -8_deg * multiplier, SWING_SPEED, -35);//turning to ring 3 about the right side
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED / 1.4);//getting ring 3
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED);//avoiding blue ring
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg * multiplier, TURN_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  armControlCopy->setTarget(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, DRIVE_SPEED/3);
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
  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED);//approaching 3rd mogo---- was 19.5
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
  chassis.pid_drive_set(-16_in, DRIVE_SPEED);
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
  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED);//approaching 3rd mogo --- was 19.5 before
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