#pragma once
#include "vex.h"
#include "core.h"
#include "cata_system.h"

using namespace vex;

extern brain Brain;
extern controller con;

extern inertial imu;
extern gps gps_sensor;
extern vex::distance intake_watcher;
extern optical cata_watcher;
extern pot cata_pot;

// ================ OUTPUTS ================
// Motors
extern motor left_front_front;
extern motor lft_front_back;
extern motor left_back_front;
extern motor left_back_back;

extern motor right_front_front;
extern motor right_front_back;
extern motor right_back_front;
extern motor right_back_back;

extern motor intake_combine;
extern motor intake_roller;

extern motor cata_r;
extern motor cata_l;

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;

extern CataSys cata_sys;
extern digital_out left_wing;
extern digital_out right_wing;

extern digital_out left_climb;
extern digital_out right_climb;

extern motor_group left_motors;
extern motor_group right_motors;

extern motor_group intake_motors;
extern motor_group cata_motors;

extern robot_specs_t robot_cfg;
extern MotionController drive_mc;
extern OdometryTank odom;
extern TankDrive drive_sys;
extern CataSys cata_sys;

// ================ UTILS ================

void robot_init();