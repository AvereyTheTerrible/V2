#pragma once

#include "api.h"

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(3);
// inline pros::adi::DigitalIn limit_switch('A');

inline pros::ADIPort clampCylinder('A', pros::E_ADI_DIGITAL_OUT);

inline pros::ADIPort sweeperCylinder('C', pros::E_ADI_DIGITAL_OUT);