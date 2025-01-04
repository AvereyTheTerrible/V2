#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intakeMotors {-7};
// inline pros::adi::DigitalIn limit_switch('A');
inline pros::MotorGroup armMotor{11, -13};

inline pros::ADIDigitalOut clampCylinder('A');
inline pros::ADIPort intakeCylinder('D', pros::E_ADI_DIGITAL_OUT);
inline pros::ADIPort sweeperCylinder('C', pros::E_ADI_DIGITAL_OUT);

inline pros::Optical colorSensor(19);
inline pros::Rotation rotationSensor(12);