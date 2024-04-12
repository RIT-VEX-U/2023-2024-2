#include "robot-config.h"
brain Brain;
controller con;
#define COMP_BOT

using namespace vex;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
inertial imu(PORT4);
gps gps_sensor(PORT8, -3, 5, distanceUnits::in, 0, turnType::left);
vex::distance intake_watcher(vex::PORT3);
vex::optical cata_watcher(vex::PORT15);
vex::pot cata_pot(Brain.ThreeWirePort.H);

// ================ OUTPUTS ================
// Motors
constexpr gearSetting drive_gears = gearSetting::ratio18_1;
motor left_front_front(PORT17, drive_gears, true);
motor left_front_back(PORT18, drive_gears, false);
motor left_back_front(PORT19, drive_gears, false);
motor left_back_back(PORT20, drive_gears, true);

motor right_front_front(PORT14, drive_gears, false);
motor right_front_back(PORT13, drive_gears, true);
motor right_back_front(PORT12, drive_gears, true);
motor right_back_back(PORT11, drive_gears, false);

motor intake_combine(PORT1, gearSetting::ratio18_1, false);
motor intake_roller(PORT9, gearSetting::ratio18_1, false);

motor cata_r(PORT2, gearSetting::ratio36_1, true);
motor cata_l(PORT10, gearSetting::ratio36_1, false);

// ================ Three-Wire Ports ==============
vex::pneumatics left_wing(Brain.ThreeWirePort.A); 
vex::pneumatics right_wing(Brain.ThreeWirePort.F);

vex::pneumatics stabilizer_sol(Brain.ThreeWirePort.C);
vex::pneumatics l_endgame_sol(Brain.ThreeWirePort.G);
vex::pneumatics r_endgame_sol(Brain.ThreeWirePort.D);
vex::pneumatics cata_sol(Brain.ThreeWirePort.B);

vex::digital_out vision_light(Brain.ThreeWirePort.E);

#else
// NEMO CONFIG

// ================ INPUTS ================
// Digital sensors

// Analog sensors
inertial imu(PORT10);
gps gps_sensor(PORT8, 0, 0, distanceUnits::in, 0, turnType::left);
vex::distance intake_watcher(vex::PORT3);
vex::optical cata_watcher(vex::PORT15);
vex::pot cata_pot(Brain.ThreeWirePort.H);

// ================ OUTPUTS ================
// Motors
constexpr gearSetting drive_gears = gearSetting::ratio18_1;
motor left_front_front(PORT1, drive_gears, true);
motor left_front_back(PORT2, drive_gears, true);
motor left_back_front(PORT22, drive_gears, false); // Disabled
motor left_back_back(PORT22, drive_gears, true);   // Disabled

motor right_front_front(PORT3, drive_gears, false);
motor right_front_back(PORT4, drive_gears, false);
motor right_back_front(PORT22, drive_gears, true); // Disabled
motor right_back_back(PORT22, drive_gears, false); // Disabled

motor intake_combine(PORT22, gearSetting::ratio18_1, false); // Disabled
motor intake_roller(PORT22, gearSetting::ratio18_1, false);  // Disabled

motor cata_r(PORT22, gearSetting::ratio36_1, true);  // Disabled
motor cata_l(PORT22, gearSetting::ratio36_1, false); // Disabled

vex::digital_out left_wing(Brain.ThreeWirePort.G);
vex::digital_out right_wing(Brain.ThreeWirePort.F);

vex::digital_out left_climb(Brain.ThreeWirePort.A);
vex::digital_out right_climb(Brain.ThreeWirePort.B);

#endif

// ================ SUBSYSTEMS ================

motor_group left_motors(left_front_front, left_front_back, left_back_front, left_back_back);
motor_group right_motors(right_front_front, right_front_back, right_back_front, right_back_back);
motor_group intake_motors = {intake_combine, intake_roller};
motor_group cata_motors(cata_l, cata_r);

std::map<std::string, motor &> motor_names = {
  {"left ff", left_front_front},
  {"left fb", left_front_back},
  {"left bf", left_back_front},
  {"left bb", left_back_back},

  {"right ff", right_front_front},
  {"right fb", right_front_back},
  {"right bf", right_back_front},
  {"right bb", right_back_back},

  {"cata L", cata_l},
  {"cata R", cata_r},

  {"intake H", intake_combine},
  {"intake L", intake_roller},

};

PID::pid_config_t drive_pid_cfg = {.p = .2, .i = 0.0, .d = .01, .deadband = 0.5, .on_target_time = .1};
PID drive_pid(drive_pid_cfg);

PID::pid_config_t turn_pid_cfg = {
  // .p = 0.05,
  // .i = 0.00,
  // .d = 0.0020,
  // .deadband = 3,
  // .on_target_time = 0.1
};

PID::pid_config_t drive_mc_pid_cfg = {
  .p = 0.1,
  // .i = 0,
  .d = 0.005,
  .deadband = 2,
  .on_target_time = 0.01,
};

FeedForward::ff_config_t drive_mc_ff_cfg = {
  .kS = 0.03, .kV = 0.0145, .kA = 0.001,
  // .kG = 0,
};

// FAST linear motion profile
MotionController::m_profile_cfg_t drive_mc_fast_cfg{
  .max_v = 55,  // Max 55 in/sec (at 100%)
  .accel = 180, // Max 200 in/sec^2 (at 100%)
  .pid_cfg = drive_mc_pid_cfg,
  .ff_cfg = drive_mc_ff_cfg};
MotionController drive_mc_fast(drive_mc_fast_cfg);

// SLOW linear motion profile
MotionController::m_profile_cfg_t drive_mc_slow_cfg = {
  .max_v = 20,  // in/sec
  .accel = 100, // in/sec^2
  .pid_cfg = drive_mc_pid_cfg,
  .ff_cfg = drive_mc_ff_cfg};
MotionController drive_mc_slow(drive_mc_slow_cfg);

MotionController::m_profile_cfg_t turn_mc_cfg{
  .max_v = 520, // 520 deg/sec max
  .accel = 400, // 900 deg/sec2 max
  .pid_cfg =
    PID::pid_config_t{
      .p = 0.06,
      // .i = 0,
      .d = 0.001,
      .deadband = 6, // Get within X degrees of the setpoint
      .on_target_time = .01,
    },
  .ff_cfg = FeedForward::ff_config_t{
    .kS = 0.02,
    .kV = 0.00105, // 0.002,
    .kA = 0.0002,
    // .kG = 0,
  }};
MotionController turn_mc{turn_mc_cfg};

robot_specs_t robot_cfg = {
  .robot_radius = 8,         // inches
  .odom_wheel_diam = 4.0125, // inches
  .odom_gear_ratio = .6667,
  .dist_between_wheels = 10.45, // inches
  .drive_correction_cutoff = 4, // inches
  .drive_feedback = &drive_mc_fast,
  .turn_feedback = &turn_mc, // new PID(turn_pid_cfg),
  .correction_pid = (PID::pid_config_t){
    .p = .04,
    .d = .003
  }};

PID::pid_config_t pc = {
  .p = 1,
  // .i = 2,
  .deadband = 2,
  .on_target_time = 0.3};

FeedForward::ff_config_t ffc = {.kG = -2};

PIDFF cata_pid(pc, ffc);

OdometryTank odom{left_motors, right_motors, robot_cfg, &imu};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);
CataSys cata_sys(
  intake_watcher, cata_pot, cata_watcher, cata_motors, intake_combine, intake_roller, cata_pid, DropMode::Unnecessary, l_endgame_sol, r_endgame_sol, cata_sol
);

// ================ UTILS ================
std::vector<screen::Page *> pages;

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous
 * are started.
 */
void robot_init() {

  pages = {
    new AutoChooser({"Auto 1", "Auto 2", "Auto 3", "Auto 4"}),
    new screen::OdometryPage(odom, 12, 12, true),
    cata_sys.Page(),
    new screen::StatsPage(motor_names),
  };

  screen::start_screen(Brain.Screen, pages, 2);
  imu.calibrate();
 
  l_endgame_sol.set(false);
  r_endgame_sol.set(false);
  cata_sol.set(false);
  stabilizer_sol.close();

  gps_sensor.calibrate();
}
