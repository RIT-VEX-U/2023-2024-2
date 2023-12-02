#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "automation.h"
#include "robot-config.h"
#include "vex.h"
#include <atomic>

#define Tank

void tuning()
{
    bool done = false;
    odom.set_position({.x=18, .y=125, .rot=204});
    while (con.ButtonUp.pressing()) {
        // if(!done && drive_sys.drive_forward(12, directionType::fwd, .5))
        //     done = true;

        // if (!done && drive_sys.turn_to_heading(330, .5))
        //     done = true;
        
        if (!done && drive_sys.pure_pursuit(PurePursuit::Path({
                {.x=18, .y=125},
                {.x=38, .y=134},
                {.x=98, .y=130},
                }, 6), directionType::rev, .5))
            done = true;

        pose_t pos = odom.get_position();
        printf("x: %.2f, y: %.2f, r: %.2f\n", pos.x, pos.y, pos.rot);
        vexDelay(10);
    }
}

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // autonomous();
    // vexDelay(1000);

    // while (imu.isCalibrating()) // || gps_sensor.isCalibrating())
    // {
    //     vexDelay(20);
    // }

    static bool enable_matchload = false;

    left_wing.set(false);
    right_wing.set(false);

    // Controls:
    // Cata: Hold L1 (Not on rising edge)
    // -- Don't shoot until there's a ball
    // -- Preload
    // Intake:
    // -- R1 IN
    // -- R2 OUT
    // -- B - 2 intake pistons

    // SUBJECT TO CHANGE!
    // Wings: DOWN
#ifdef COMP_BOT
    con.ButtonL2.pressed([]() { cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL2.released([]() { cata_sys.send_command(CataSys::Command::StopFiring); });

    con.ButtonR1.pressed([]() { cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR1.released([](){ cata_sys.send_command(CataSys::Command::StopIntake); });

    con.ButtonR2.pressed([]() { cata_sys.send_command(CataSys::Command::IntakeOut); });
    con.ButtonR2.released([](){ cata_sys.send_command(CataSys::Command::StopIntake); });

    con.ButtonL1.pressed([]() { cata_sys.send_command(CataSys::Command::IntakeHold); });
    con.ButtonL1.released([](){ cata_sys.send_command(CataSys::Command::StopIntake);});

    con.ButtonB.pressed([]() { 
        static bool wing_isdown = false;
        wing_isdown = !wing_isdown;
        left_wing.set(wing_isdown);
        right_wing.set(wing_isdown); 
    });

    con.ButtonA.pressed([]() { enable_matchload = !enable_matchload; });
    pose_t start_pose = { .x = 16, .y = 144 - 16, .rot = 135 };

    // CommandController cc{
    //     new RepeatUntil(
    //         {
    //             odom.SetPositionCmd(start_pose),
    //             cata_sys.IntakeFully(),
    //             drive_sys.DriveForwardCmd(4, directionType::rev),
    //             cata_sys.Fire(),
    //             drive_sys.DriveForwardCmd(4, directionType::fwd),

    //         },
    //         10),
    //     // drive_sys.DriveForwardCmd(36, vex::directionType::rev, 0.9),
    //     // drive_sys.TurnToHeadingCmd(-90),
    //     // drive_sys.DriveToPointCmd({.x = 27, .y = 18}, vex::fwd)
    // };
    // cc.add_cancel_func([]()
    //                    { return con.ButtonA.pressing(); });
    // cc.run();
    // while(true){
    // vexDelay(1000);
    // }
    // return;

#endif
  // ================ INIT ================
    while (true) {
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
#endif

        // tuning();

        matchload_1([&](){ return enable_matchload;}); // Toggle
        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}