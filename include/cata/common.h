#pragma once
enum class DropMode {
    Required,
    Unnecessary,
};

// Intake Params
const double inake_enable_lower_threshold = 90;
const double intake_enable_upper_threshold = 110;

const double intake_upper_outer_volt = 12.0;
const double intake_lower_outer_volt = 9.0;

const double intake_upper_volt = 12;
const double intake_lower_volt = 9.0;

const double intake_upper_volt_hold = 12.0;
const double intake_lower_volt_hold = 9.0;

const double intake_sensor_dist_mm = 150;

const double intake_drop_seconds = 0.5;

// Cata Params
const double cata_target_charge = 98; 
const double done_firing_angle = 110;

const double intake_drop_seconds_until_enable = 0.25;
const double fire_voltage = 12.0;
