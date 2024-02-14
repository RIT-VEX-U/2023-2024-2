#include "competition/opcontrol.h"
#include "competition/autonomous.h"
#include "automation.h"
#include "robot-config.h"
#include "vex.h"
#include "vision.h"
#include <atomic>
#include "tuning.h"

#define Tank

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // ================ TUNING CODE (Disable when not testing) ================
    testing();

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

    con.ButtonL2.pressed([]() { cata_sys.send_command(CataSys::Command::StartFiring); });
    // con.ButtonL2.released([]() { cata_sys.send_command(CataSys::Command::StopFiring); });

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

    // con.ButtonUp.pressed([]() { enable_matchload = !enable_matchload; });
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

    // personal debug button >:]
    con.ButtonRight.pressed([](){
        vision_light.set(!vision_light.value());
        // gps_localize_stdev();
    });

    vision_light.set(false);

    static std::atomic<bool> brake_mode_toggled(false);
    con.ButtonX.pressed([](){
        brake_mode_toggled = !brake_mode_toggled;
    });

    con.ButtonUp.pressed([](){
        cata_sys.send_command(CataSys::Command::ToggleCata);
    });


    // ================ PERIODIC ================
    while (true) {
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        if(!disable_drive && !enable_matchload)
        {
            if(brake_mode_toggled)
                drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::Smart);
            else
                drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);
        }

        if(!disable_drive && enable_matchload)
        {
            double l = con.Axis3.position() / 100.0;
            double r = con.Axis1.position() / 100.0;
            drive_sys.drive_arcade(l, 0);
        }

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        if(!disable_drive)
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
#endif


        // matchload_1([&](){ return enable_matchload;}); // Toggle
        // static timer matchload_tmr;
        // if(enable_matchload)
        // {
        //     AutoCommand *drive1 = drive_sys.DriveForwardCmd(8, directionType::rev, 0.4)->withTimeout(1);
        //     AutoCommand *drive2 = drive_sys.DriveForwardCmd(8, directionType::fwd, 0.4)->withTimeout(1);
        //     AutoCommand *delay = new DelayCommand(350);
        //     CommandController cmd{
        //         drive1, drive2, delay
        //     };
        //     cmd.run();
        //     // Clean up bc memory is crazy
        //     delete drive1;
        //     delete drive2;
        //     delete delay;
        // }

        // printf("x: %f\n", gps_sensor.xPosition(distanceUnits::in));
        // static VisionTrackTriballCommand viscmd;
        // static VisionObjectExists existscmd(vision_filter_s{
        //             .min_area = 8000,
        //             .max_area = 100000,
        //             .aspect_low = 0.5,
        //             .aspect_high = 2,
        //             .min_x = 0,
        //             .max_x = 320,
        //             .min_y = 0,
        //             .max_y = 240,
        //     });

        // printf("exists? %d\n", existscmd.test());

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


        // cam.takeSnapshot(TRIBALL);
        // printf("I: %f, N: %d, X: %d, Y: %d, A: %d, Ratio: %f\n", intake_watcher.objectDistance(distanceUnits::mm), cam.objectCount,
        //     cam.largestObject.centerX, cam.largestObject.centerY, 
        //     cam.largestObject.width * cam.largestObject.height,
        //     (double)cam.largestObject.width / (double)cam.largestObject.height);
        // pose_t pos = odom.get_position();
        // printf("X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        // printf("GPS X: %.2f, Y: %.2f, R: %.2f Q: %d\n", 
        //     gps_sensor.xPosition(distanceUnits::in)+72, 
        //     gps_sensor.yPosition(distanceUnits::in)+72, 
        //     gps_sensor.heading(), gps_sensor.quality());

        


        // Controls
        // Intake

        vexDelay(10);
    }


}

void testing()
{
    // ================ AUTONOMOUS TESTING ================
    // autonomous(); 
    cata_sys.send_command(CataSys::Command::ToggleCata);

    while(imu.isCalibrating() || gps_sensor.isCalibrating()) {vexDelay(20);}

    static std::atomic<bool> disable_drive(false);

    // // ================ ODOMETRY TESTING ================

    // // Reset encoder odometry to 0,0,90 with button X
    con.ButtonX.pressed([](){
        odom.set_position({28, 47, 0});
    });

    // // Test localization with button Y
    con.ButtonY.pressed([](){
        disable_drive = true;
        GPSLocalizeCommand().run();
        disable_drive = false;
    });

    // // ================ DRIVE TESTING ================

    // // While *holding* button A, tune drive-to-point
    con.ButtonA.pressed([](){
        disable_drive = true;

        FunctionCondition end_con([](){
            return !con.ButtonA.pressing();
        });
        CommandController{
            drive_sys.DriveToPointCmd({0, 48}, directionType::fwd)->withCancelCondition(&end_con)
        }.run();

        disable_drive = false;
    });

    // // While *holding* button B, tune turn-to-heading
    con.ButtonB.pressed([](){
        disable_drive = true;

        FunctionCondition end_con([](){
            return !con.ButtonB.pressing();
        });
        CommandController{
            drive_sys.TurnToHeadingCmd(turn_mc, 270)->withCancelCondition(&end_con)
        }.run();

        disable_drive = false;
    });

    // // ================ VISION TESTING ================
    con.ButtonRight.pressed([](){
        vision_light.set(!vision_light.value());
    });

    con.ButtonLeft.pressed([](){
        cata_sys.send_command(CataSys::Command::ToggleCata);
    });

    while(true)
    {
        // ================ Controls ================
        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        if(!disable_drive)
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);

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
        // printf("Pot: %f\n", cata_pot.angle(vex::deg));

        vexDelay(20);
    }
}