#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "automation.h"
#include "robot-config.h"
#include "vex.h"
#include "vision.h"
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

    // while (imu.isCalibrating() || gps_sensor.isCalibrating())
    // {
    //     vexDelay(20);
    // }

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

    con.ButtonUp.pressed([]() { enable_matchload = !enable_matchload; });
    pose_t start_pose = { .x = 16, .y = 144 - 16, .rot = 135 };

    static std::atomic<bool> disable_drive(false);

    con.ButtonA.pressed([](){
        // Turn Right
        disable_drive = true;
        right_motors.spin(directionType::rev, 5, volt);
        left_motors.spin(directionType::fwd, 3, volt);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        disable_drive = false;
        
    });

    con.ButtonLeft.pressed([](){
        // Turn Left
        disable_drive = true;
        right_motors.spin(directionType::fwd, 3, volt);
        left_motors.spin(directionType::rev, 5, volt);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        disable_drive = false;
    });

    con.ButtonDown.pressed([](){
        // Climb
        static bool isUp = false;
        left_climb.set(isUp = !isUp);
        right_climb.set(isUp);
    }); 

    con.ButtonRight.pressed([](){
        vision_light.set(!vision_light.value());
    });

    vision_light.set(false);

    static std::atomic<bool> brake_mode_toggled(false);
    con.ButtonX.pressed([](){
        brake_mode_toggled = !brake_mode_toggled;
    });

  // ================ INIT ================
    while (true) {
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        if(!disable_drive)
        {
            if(brake_mode_toggled)
                drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::Smart);
            else
                drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);
        }

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        if(!disable_drive)
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
#endif

        // tuning();

        matchload_1([&](){ return enable_matchload;}); // Toggle

        // printf("x: %f\n", gps_sensor.xPosition(distanceUnits::in));
        static VisionTrackTriballCommand viscmd;
        static VisionObjectExists existscmd(vision_filter_s{
                    .min_area = 8000,
                    .max_area = 100000,
                    .aspect_low = 0.5,
                    .aspect_high = 2,
                    .min_x = 0,
                    .max_x = 320,
                    .min_y = 0,
                    .max_y = 240,
            });

        printf("exists? %d\n", existscmd.test());

        // if(con.ButtonB.pressing())
        // {
        //     if(intake_watcher.objectDistance(distanceUnits::mm) > 100.0)
        //         viscmd.run();
        //     else
        //         drive_sys.stop();

        //     cata_sys.send_command(CataSys::Command::IntakeHold);
        // }

        if (con.ButtonY.pressing())
        {
            static FunctionCommand* reset_gps_cmd = gps_reset();
            reset_gps_cmd->run();
        }


        cam.takeSnapshot(TRIBALL);
        printf("I: %f, N: %d, X: %d, Y: %d, A: %d, Ratio: %f\n", intake_watcher.objectDistance(distanceUnits::mm), cam.objectCount,
            cam.largestObject.centerX, cam.largestObject.centerY, 
            cam.largestObject.width * cam.largestObject.height,
            (double)cam.largestObject.width / (double)cam.largestObject.height);
        // pose_t pos = odom.get_position();
        // printf("X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        // printf("GPS X: %.2f, Y: %.2f, R: %.2f Q: %d\n", 
        //     gps_sensor.xPosition(distanceUnits::in)+72, 
        //     gps_sensor.yPosition(distanceUnits::in)+72, 
        //     gps_sensor.heading(), gps_sensor.quality());

        // if (gps_sensor.quality() > 95)
        // {
        //     odom.set_position({
        //         .x=gps_sensor.xPosition(distanceUnits::in) + 72, 
        //         .y=gps_sensor.yPosition(distanceUnits::in) + 72,
        //         .rot=gps_sensor.heading(),
        //     });
        // }

        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}