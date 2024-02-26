#include "tuning.h"
#include "automation.h"
#include "core.h"
#include "math.h"
#include "robot-config.h"

#define ENC_IN(enc) (enc.position(rev) * PI * robot_cfg.odom_wheel_diam / robot_cfg.odom_gear_ratio)
#define ENC_DIFF_IN(left, right) (fabs(ENC_IN(left) - ENC_IN(right)) / 2.0)

double stored_avg = 0;
double stored_num = 0;
double continuous_avg(double updateval) {
  stored_num++;

  if (stored_num < 1) {
    stored_avg = 0.0;
  } else {
    stored_avg = (stored_avg * (stored_num - 1) / stored_num) + (updateval / stored_num);
  }
  return stored_avg;
}

void reset_avg_counter() {
  stored_avg = 0;
  stored_num = 0;
}

// Odometry Tuning
void tune_odometry_gear_ratio_right_wheel() {
  if (con.ButtonA.pressing()) {
    // SET THESE BACK TO LEFT ENC RIGHT ENC
    right_motors.resetPosition();
  }
  double ratio = right_motors.position(rev);

  con.Screen.clearScreen();
  con.Screen.setCursor(1, 1);
  con.Screen.print("turn right odom wheel 1 rev");
  con.Screen.setCursor(2, 1);
  con.Screen.print("ratio: %f", ratio);
  printf("ratio: %f", ratio);
}

void tune_odometry_wheel_diam() {
  if (con.ButtonA.pressing()) {
    // SET THESE BACK TO LEFT ENC RIGHT ENC
    // left_motors.resetPosition();
    // right_motors.resetPosition();
    left_motors.resetPosition();
    right_motors.resetPosition();
  }
  double avg = (fabs(left_motors.position(rev)) + fabs(right_motors.position(rev))) / 2.0;
  avg /= robot_cfg.odom_gear_ratio;
  // double avg = (fabs(left_motors.position(rev)) +
  // fabs(right_motors.position(rev))) / 2.0;
  double diam = 0;
  if (fabs(avg) < .1) {
    diam = 0;
  } else {
    diam = 100.0 / (avg * PI);
  }

  con.Screen.clearScreen();
  con.Screen.setCursor(1, 1);
  con.Screen.print("Push robot 100 inches");
  con.Screen.setCursor(2, 1);
  con.Screen.print("Diam: %f", diam);
  printf("Diam: %f\n", diam);
}

void tune_odometry_wheelbase() {
  int times_to_turn = 5;
  if (con.ButtonA.pressing()) {
    left_motors.resetPosition();
    right_motors.resetPosition();
    // left_motors.resetPosition();
    // right_motors.resetPosition();
  }
  double radius =
    ENC_DIFF_IN(left_motors, right_motors) / ((double)times_to_turn * 2 * PI); // radius = arclength / theta
  // double radius = ENC_DIFF_IN(left_motors, right_motors) /
  // ((double)times_to_turn * 2 * PI); // radius = arclength / theta

  double wheelbase = 2 * radius;

  con.Screen.clearScreen();
  con.Screen.setCursor(1, 1);
  con.Screen.print("Turn the robot in place %d times", times_to_turn);
  con.Screen.setCursor(2, 1);
  con.Screen.print("Wheelbase: %f", wheelbase);
  printf("Wheelbase: %f\n", wheelbase);
}

// Drive Tuning
void tune_drive_ff_ks(DriveType dt) {
  static timer tmr;
  static double test_pct = 0.0;
  static bool new_press = true;
  static bool done = false;

  if (con.ButtonA.pressing()) {
    if (new_press) {
      // Initialize the function once
      tmr.reset();
      left_motors.resetPosition();
      right_motors.resetPosition();
      // left_motors.resetPosition();
      // right_motors.resetPosition();
      test_pct = 0.0;
      done = false;
      new_press = false;
    }

    if (done || (fabs(left_motors.position(rev)) + fabs(right_motors.position(rev))) > 0) {
      con.Screen.clearScreen();
      con.Screen.setCursor(1, 1);
      con.Screen.print("kS: %f", test_pct);
      printf("kS: %f\n", test_pct);
      done = true;
      return;
    } else {
      con.Screen.clearScreen();
      con.Screen.setCursor(1, 1);
      con.Screen.print("Running...");
    }

    if (tmr.time() > 500) {
      test_pct += 0.01;
      tmr.reset();
    }

    if (dt == DRIVE)
      drive_sys.drive_tank(test_pct, test_pct);
    else if (dt == TURN)
      drive_sys.drive_tank(test_pct, -test_pct);
  } else {
    drive_sys.stop();
    new_press = true;
  }
}

void tune_drive_ff_kv(DriveType dt, double ks) {
  static bool new_press = true;
  static timer tmr;

  static int start_delay_ms = 2000;

  if (con.ButtonA.pressing()) {
    if (new_press) {
      tmr.reset();
      reset_avg_counter();
      new_press = false;
    }

    double vel = 0;

    if (dt == DRIVE) {
      drive_sys.drive_tank(0.5, 0.5);
      vel = odom.get_speed();
    } else if (dt == TURN) {
      drive_sys.drive_tank(0.5, -0.5);
      vel = odom.get_angular_speed_deg();
    }

    double kv = 0;
    if (tmr.time(msec) > start_delay_ms)
      kv = (0.5 - ks) / continuous_avg(vel);

    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("kV: %f", kv);
    printf("kV: %f\n", kv);
  } else {
    drive_sys.stop();
    new_press = true;
  }
}

void tune_drive_pid(DriveType dt) {
  static bool done = false;

  if (con.ButtonB.pressing())
    odom.set_position();

  if (con.ButtonA.pressing()) {
    auto pos = odom.get_position();
    printf("%.2f, %.2f, %.2f, ", pos.x, pos.y, pos.rot);
    printf("accel: %2f\n", odom.get_accel());

    if (dt == DRIVE && (done || drive_sys.drive_to_point(0, 48, fwd, drive_mc_fast))) {

      printf("Finished\n");
      fflush(stdout);
      done = true;
    }

    if (dt == TURN && (done || drive_sys.turn_to_heading(180, *robot_cfg.turn_feedback, 1.0))) {
      printf("Finished\n");
      fflush(stdout);
      done = true;
    }
  } else {
    drive_sys.drive_arcade(con.Axis3.position() / 100.0, con.Axis1.position() / 100.0);
    drive_sys.reset_auto();
    if (con.ButtonB.pressing())
      odom.set_position();
    done = false;
  }

  auto pos = odom.get_position();
  printf("%.2f, %.2f, %.2f\n", pos.x, pos.y, pos.rot);
}

void tune_drive_motion_maxv(DriveType dt) {
  static bool new_press = true;

  if (con.ButtonA.pressing()) {
    if (new_press) {
      reset_avg_counter();
      new_press = false;
    }

    double maxv = 0;

    if (dt == DRIVE) {
      drive_sys.drive_tank(1.0, 1.0);
      maxv = continuous_avg(odom.get_speed());
    } else if (dt == TURN) {
      drive_sys.drive_tank(1.0, -1.0);
      maxv = continuous_avg(odom.get_angular_speed_deg());
    }

    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("maxV: %f", maxv);
    printf("maxV: %f\n", maxv);
  } else {
    drive_sys.stop();
    new_press = true;
  }
}

void tune_drive_motion_accel(DriveType dt, double maxv) {
  static bool done = false;
  static bool new_press = true;
  static double accel = 0;

  if (con.ButtonA.pressing()) {
    if (new_press) {
      reset_avg_counter();
      done = false;
      new_press = false;
    }

    double vel = 0;

    if (!done && dt == DRIVE) {
      vel = odom.get_speed();
      accel = odom.get_accel();
    } else if (!done) {
      static vex::timer tmr;
      static uint64_t last_time = tmr.systemHighResolution();
      static double last_rot = imu.rotation(deg);
      static double last_vel = 0;

      double delta = (tmr.systemHighResolution() - last_time) / 1000000.0;

      vel = (imu.rotation(deg) - last_rot) / delta;
      accel = (vel - last_vel) / delta;

      last_time = tmr.systemHighResolution();
      last_rot = imu.rotation(deg);
      last_vel = vel;

      // vel = odom.get_angular_speed_deg();
      // accel = odom.get_angular_accel_deg();
    }

    if (done || vel >= maxv) {
      con.Screen.clearScreen();
      con.Screen.setCursor(1, 1);
      con.Screen.print("Accel: %f", accel);
      printf("Accel: %f\n", accel);
      done = true;
      drive_sys.stop();
      return;
    }

    con.Screen.clearScreen();
    con.Screen.setCursor(1, 1);
    con.Screen.print("Running...");

    if (dt == DRIVE)
      drive_sys.drive_tank(1.0, 1.0);
    else
      drive_sys.drive_tank(1.0, -1.0);
  } else {
    drive_sys.stop();
    new_press = true;
  }
}