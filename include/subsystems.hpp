#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intakeMotors{20, -10};
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::ADIDigitalOut clampCylinder('B');
inline pros::ADIPort intakeCylinder('D', pros::E_ADI_DIGITAL_OUT);
inline pros::ADIPort sweeperCylinder('C', pros::E_ADI_DIGITAL_OUT);

inline pros::Optical colorSensor(19);