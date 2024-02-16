#include "competition/opcontrol.h"
#include "automation.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "tuning.h"
#include "vex.h"
#include "vision.h"
#include <atomic>

#define Tank

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // ================ TUNING CODE (Disable when not testing) ================
    // testing();

  // ================ INIT ================
  static bool enable_matchload = false;

  left_wing.set(false);
  right_wing.set(false);

  // Controls:
  // Catapult:
  // -- L2 - Fire (Don't if ball isn't detected)
  // Intake:
  // -- L1 - Intake & Hold
  // -- R1 - In
  // -- R2 - Out
  // Misc:
  // -- B - both wings (toggle)
  // -- Down - Climb Pistons (Single deploy)
  // -- Up - Auto match load
  // -- A - quick turn (Right)
  // -- Left - quick turn (Left)

  con.ButtonL2.pressed(
      []() { cata_sys.send_command(CataSys::Command::StartFiring); });
  // con.ButtonL2.released([]() {
  // cata_sys.send_command(CataSys::Command::StopFiring); });

  con.ButtonR1.pressed(
      []() { cata_sys.send_command(CataSys::Command::IntakeIn); });
  con.ButtonR1.released(
      []() { cata_sys.send_command(CataSys::Command::StopIntake); });

  con.ButtonR2.pressed(
      []() { cata_sys.send_command(CataSys::Command::IntakeOut); });
  con.ButtonR2.released(
      []() { cata_sys.send_command(CataSys::Command::StopIntake); });

  con.ButtonL1.pressed(
      []() { cata_sys.send_command(CataSys::Command::IntakeHold); });
  con.ButtonL1.released(
      []() { cata_sys.send_command(CataSys::Command::StopIntake); });

  con.ButtonB.pressed([]() {
    static bool wing_isdown = false;
    wing_isdown = !wing_isdown;
    left_wing.set(wing_isdown);
    right_wing.set(wing_isdown);
  });

  con.ButtonUp.pressed([]() { enable_matchload = !enable_matchload; });
  pose_t start_pose = {.x = 16, .y = 144 - 16, .rot = 135};

  static std::atomic<bool> disable_drive(false);

  con.ButtonA.pressed([]() {
    // Turn Right
    disable_drive = true;
    right_motors.spin(directionType::rev, 5, volt);
    left_motors.spin(directionType::fwd, 3, volt);
    vexDelay(150);
    // right_motors.stop(brakeType::coast);
    // left_motors.stop(brakeType::coast);
    disable_drive = false;
  });

  con.ButtonLeft.pressed([]() {
    // Turn Left
    disable_drive = true;
    right_motors.spin(directionType::fwd, 3, volt);
    left_motors.spin(directionType::rev, 5, volt);
    vexDelay(150);
    // right_motors.stop(brakeType::coast);
    // left_motors.stop(brakeType::coast);
    disable_drive = false;
  });

  con.ButtonDown.pressed([]() {
    // Climb
    static bool isUp = false;
    left_climb.set(isUp = !isUp);
    right_climb.set(isUp);
  });

  // personal debug button >:]
  con.ButtonRight.pressed([]() {
    vision_light.set(!vision_light.value());
    // gps_localize_stdev();
  });

  vision_light.set(false);

  static std::atomic<bool> brake_mode_toggled(false);
  con.ButtonX.pressed([]() { brake_mode_toggled = !brake_mode_toggled; });

  // ================ PERIODIC ================
  while (true) {
#ifdef Tank
    double l = con.Axis3.position() / 100.0;
    double r = con.Axis2.position() / 100.0;
    if (!disable_drive && !enable_matchload) {
      if (brake_mode_toggled)
        drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::Smart);
      else
        drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);
    }

    if (!disable_drive && enable_matchload) {
      double l = con.Axis3.position() / 100.0;
      double r = con.Axis1.position() / 100.0;
      drive_sys.drive_arcade(l, 0);
    }

#else

    double f = con.Axis3.position() / 100.0;
    double s = con.Axis1.position() / 100.0;
    if (!disable_drive)
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
#endif

    // matchload_1([&](){ return enable_matchload;}); // Toggle
    vexDelay(10);
  }
}

void testing() {
  // ================ AUTONOMOUS TESTING ================
  autonomous();
  cata_sys.send_command(CataSys::Command::ToggleCata);

  while (imu.isCalibrating() || gps_sensor.isCalibrating()) {
    vexDelay(20);
  }

  static std::atomic<bool> disable_drive(false);

  // // ================ ODOMETRY TESTING ================

  // // Reset encoder odometry to 0,0,90 with button X
  con.ButtonX.pressed([]() { odom.set_position(); });

  // // Test localization with button Y
  con.ButtonY.pressed([]() {
    disable_drive = true;
    vexDelay(500); // Settle first
    GPSLocalizeCommand(RED).run();
    disable_drive = false;
  });

  con.ButtonA.pressed([]() {
    disable_drive = true;
    CommandController{drive_sys.DriveToPointCmd({.x = 0, .y = 48})
                          ->withCancelCondition(new FunctionCondition([]() {
                            return con.ButtonA.pressing() == false;
                          }))}
        .run();
    disable_drive = false;
  });

  // // ================ VISION TESTING ================
  con.ButtonRight.pressed([]() { vision_light.set(!vision_light.value()); });

  con.ButtonLeft.pressed(
      []() { cata_sys.send_command(CataSys::Command::ToggleCata); });

  while (true) {

    // ================ Controls ================
    double f = con.Axis3.position() / 100.0;
    double s = con.Axis1.position() / 100.0;
    if (!disable_drive)
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);

        // ================ Drive Tuning =================
        static bool done_a = false;
        if(con.ButtonA.pressing() && !done_a)
        {
            disable_drive = true;
            done_a = drive_sys.drive_to_point(0, 48, directionType::fwd);
        } else if(!con.ButtonA.pressing())
        {
            done_a = false;
        }

        // ================ Debug Print Statements ================
        pose_t odom_pose = odom.get_position();
        pose_t gps_pose = GPSLocalizeCommand::get_pose_rotated();

        // printf("ODO: {%.2f, %.2f, %.2f} | GPS: {%.2f, %.2f, %.2f}\n",
        //         odom_pose.x, odom_pose.y, odom_pose.rot, gps_pose.x, gps_pose.y, gps_pose.rot);

        auto objs = vision_run_filter(TRIBALL);
        int n=objs.size(), x=0, y=0, a=0;
        if(n > 0)
        {
            x = objs[0].centerX;
            y = objs[0].centerY;
            a = objs[0].width * objs[0].height;
        }
        // printf("CAM: N:%d | {%d, %d}, A:%d\n", n, x, y, a);
        printf("Pot: %f\n", cata_pot.angle(vex::deg));

        vexDelay(20);
    }

    // ================ Debug Print Statements ================
    pose_t odom_pose = odom.get_position();
    pose_t gps_pose = GPSLocalizeCommand::get_pose_rotated();

    // printf("ODO: {%.2f, %.2f, %.2f} | GPS: {%.2f, %.2f, %.2f}\n",
    //         odom_pose.x, odom_pose.y, odom_pose.rot, gps_pose.x, gps_pose.y,
    //         gps_pose.rot);

    auto objs = vision_run_filter(TRIBALL);
    int n = objs.size(), x = 0, y = 0, a = 0;
    if (n > 0) {
      x = objs[0].centerX;
      y = objs[0].centerY;
      a = objs[0].width * objs[0].height;
    }
    // printf("CAM: N:%d | {%d, %d}, A:%d\n", n, x, y, a);
    // printf("Pot: %f\n", cata_pot.angle(vex::deg));

    vexDelay(20);
  }