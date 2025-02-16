#pragma once

#include "api.h"
#include "okapi/api.hpp"

// Your motors, sensors, etc. should go here.  Below are examples


inline pros::Motor intakeMotors {-3};
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::MotorGroup armMotor{11, -13};

inline pros::ADIDigitalOut clampCylinder('H');
inline pros::ADIDigitalOut sweeperCylinder('G');

inline pros::Rotation rotationSensor(12);