#pragma once
#include "vex.h"
#include "core.h"
#include "cata_system.h"
#define COMP_BOT

using namespace vex;

extern brain Brain;
extern controller con;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern inertial imu;
extern gps gps_sensor;
extern vex::optical cata_watcher;
extern vex::distance intake_watcher;

// ================ OUTPUTS ================
// Motors
extern motor intake_combine;
extern motor intake_roller;

extern motor_group left_motors, right_motors;
extern motor_group cata_motors;

// ================ SUBSYSTEMS ================
extern robot_specs_t robot_cfg;
extern OdometryTank odom;
extern TankDrive drive_sys;

// extern MotionController turn_mc;
extern MotionController drive_mc;

extern CataSys cata_sys;
extern vex::digital_out left_wing;
extern vex::digital_out right_wing;

extern vex::digital_out left_climb;
extern vex::digital_out right_climb;


#else
// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern inertial imu;
extern gps gps_sensor;


// ================ OUTPUTS ================
// Motors
extern motor intake_combine;
extern motor intake_roller;

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;
#endif

// ================ UTILS ================

void robot_init();