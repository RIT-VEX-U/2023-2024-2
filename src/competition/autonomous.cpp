#include "competition/autonomous.h"
#include "robot-config.h"
#include "core.h"
#include "automation.h"
#include "vision.h"
#include <functional>

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

#define SIDE RED

enum Side
{
    LEFT, RIGHT
};

class WingCmd : public AutoCommand
{
    public:
    WingCmd(Side s, bool deploy_down) : s(s), deploy_down(deploy_down)
    { }

    bool run() override
    { 
        if(s == LEFT)
        {
            if(deploy_down)
                left_wing.set(1);
            else
                left_wing.set(0);
        } else
        {
            if(deploy_down)
                right_wing.set(1);
            else
                right_wing.set(0);
        }
        return true; 
    }

    Side s;
    bool deploy_down;
};

/**
 * Main entrypoint for the autonomous period
 */
void autonomous()
{
    while(imu.isCalibrating() || gps_sensor.isCalibrating()){vexDelay(100);}
    scoreAutoFull();
    // skills();
}

bool light_on()
{
    vision_light.set(true);
    return true;
}

bool light_off()
{
    vision_light.set(false);
    return true;
}

class DebugCommand : public AutoCommand
{
    public:

    bool run() override
    {
        drive_sys.stop();
        cata_sys.send_command(CataSys::Command::StopIntake);
        cata_sys.send_command(CataSys::Command::ToggleCata);
        pose_t pos = odom.get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f, ", pos.x, pos.y, pos.rot);
        vision_light.set(true);
        while(true)
        {
            cata_sys.send_command(CataSys::Command::StopIntake);
            double f = con.Axis3.position() / 200.0;
            double s = con.Axis1.position() / 200.0;
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
            pose_t pos = odom.get_position();
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            // printf("GPS X: %.2f, Y: %.2f, R: %.2f Q: %d\n", 
            //     gps_sensor.xPosition(distanceUnits::in)+72, 
            //     gps_sensor.yPosition(distanceUnits::in)+72, 
            //     gps_sensor.heading(), gps_sensor.quality());
            cam.takeSnapshot(TRIBALL);
            printf("X: %d, Y: %d, A: %d, Ratio: %f\n", 
                cam.largestObject.centerX, cam.largestObject.centerY, 
                cam.largestObject.width * cam.largestObject.height,
                (double)cam.largestObject.width / (double)cam.largestObject.height);
            vexDelay(100);
        }
        return false;
    }
};

void scoreAutoFull()
{
    DebugCommand *tempend = new DebugCommand();

    CommandController cmd{
        odom.SetPositionCmd({.x=54, .y=15, .rot=0}),
        new DelayCommand(500), // Wait for the catapult to go down
        // Collect Ball (after delay)
        cata_sys.IntakeToHold(),

        // Drive to goal
        drive_sys.PurePursuitCmd(PurePursuit::Path({
            {.x=54, .y=15},
            {.x=97, .y=17},
            {.x=120, .y=28},
            {.x=124, .y=38},
        }, 8), directionType::fwd, 0.45),
        drive_sys.TurnToHeadingCmd(90),
        cata_sys.Unintake(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(10, FWD)->withTimeout(1),
        cata_sys.StopIntake(),
        // new GPSLocalizeCommand(), // 126, 39, 90 ish 
        // Back up & get alliance ball
        drive_sys.DriveForwardCmd(drive_mc_slow, 4, REV),
        drive_sys.TurnToHeadingCmd(230),
        cata_sys.IntakeToHold(),
        drive_sys.PurePursuitCmd(drive_mc_slow, PurePursuit::Path({
            {.x=124, .y=43},
            {.x=124, .y=35},
            {.x=129, .y=26}
        }, 4), FWD)->withTimeout(1),
        cata_sys.WaitForHold()->withTimeout(3),

        drive_sys.DriveForwardCmd(4, REV, 0.4),
        drive_sys.TurnToHeadingCmd(48),
        drive_sys.DriveToPointCmd({131, 40}, FWD),
        drive_sys.TurnToHeadingCmd(90),
        
        // Score alliance ball
        cata_sys.Unintake(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(16, FWD)->withTimeout(1),
        cata_sys.StopIntake(),
        drive_sys.DriveForwardCmd(8, REV)->withTimeout(1),
        new GPSLocalizeCommand(SIDE),

        // Drive into position
        drive_sys.TurnToHeadingCmd(180),

        new FunctionCommand(light_on),
        drive_sys.PurePursuitCmd(drive_mc_slow, PurePursuit::Path({
            {.x=127, .y=30},
            {.x=109, .y=30},
            {.x=98, .y=40},
            {.x=98, .y=46},
        }, 8), FWD),

        drive_sys.TurnToHeadingCmd(90),

        // grab from center (closest to goal)
        // TODO make sure we don't cross the line! (Async command?)
        
        new Branch {
            new VisionObjectExists(vision_filter_s{
                    .min_area = 2000,
                    .max_area = 100000,
                    .aspect_low = 0.5,
                    .aspect_high = 2,
                    .min_x = 0,
                    .max_x = 320,
                    .min_y = 0,
                    .max_y = 240,
            }), // Maybe add filter for left/right limits?
            new InOrder{ // Object doesn't exist, big sad (turn & continue on)
                new FunctionCommand(light_on),
                drive_sys.TurnToHeadingCmd(123),
            },
            new InOrder { // Object exists, go get 'em!
                cata_sys.IntakeToHold(),
                new VisionTrackTriballCommand(),
                // Back up & score
                drive_sys.TurnToHeadingCmd(0),
                cata_sys.Unintake(),
                drive_sys.DriveForwardCmd(18, FWD, 0.8)->withTimeout(1),
                cata_sys.StopIntake(),

                // Position for next ball
                drive_sys.DriveForwardCmd(8, REV, 0.4)->withTimeout(1),
                new FunctionCommand(light_on),
                drive_sys.TurnToHeadingCmd(168, 0.4),
            }
        },
        new Branch {
            new VisionObjectExists(vision_filter_s{
                    .min_area = 2000,
                    .max_area = 100000,
                    .aspect_low = 0.5,
                    .aspect_high = 2,
                    .min_x = 0,
                    .max_x = 320,
                    .min_y = 0,
                    .max_y = 240,
            }),
            new InOrder { // Object doesn't exist, big sad (turn & continue on)
                // No change
            },
            new InOrder { // Object exists, go get 'em!
                cata_sys.IntakeToHold(),
                new VisionTrackTriballCommand(),
                drive_sys.TurnToHeadingCmd(0, 0.4),
                cata_sys.Unintake(),
                drive_sys.DriveForwardCmd(36, FWD, 0.8)->withTimeout(1),
                cata_sys.StopIntake(),
                drive_sys.DriveForwardCmd(8, REV),
            }
        },
        
        // Grab triball along wall
        drive_sys.TurnToHeadingCmd(214, 0.4),
        cata_sys.IntakeToHold(),
        new VisionTrackTriballCommand(),
        new Branch {
            new FunctionCondition([](){return intake_watcher.objectDistance(mm) < 150;}),
            new InOrder { // No ball detected
                // do later, shouldn't get here
            },
            new InOrder { // Ball detected, score it
                drive_sys.TurnToHeadingCmd(17, 0.4),
                new Async{
                    new InOrder {
                        new DelayCommand(200),
                        cata_sys.Unintake()
                    }
                },
                drive_sys.DriveForwardCmd(40, FWD, 0.65)->withTimeout(1.5),
                cata_sys.StopIntake(),
                drive_sys.DriveForwardCmd(8, REV, 0.4),
                new GPSLocalizeCommand(SIDE),
            }
        },
        
        // complete AWP
        drive_sys.TurnToHeadingCmd(230),
        drive_sys.DriveToPointCmd({.x=75, .y=34}, FWD)->withTimeout(3),
        drive_sys.DriveForwardCmd(48, FWD, 0.2)->withTimeout(3),
    };

    cmd.run();
}

void skills()
{
    FunctionCommand *intakeToCata = new FunctionCommand([](){
        drive_sys.drive_tank(0.15, 0.15);
        // Run intake in, periodically push out in case it's caught.
        // static vex::timer intake_tmr;
        // if(intake_tmr.time(sec) > 1)
        //     cata_sys.send_command(CataSys::Command::IntakeOut);
        // else if( intake_tmr.time(sec) > 1.5)
        //     intake_tmr.reset();
        // else
        //     cata_sys.send_command(CataSys::Command::IntakeIn);

        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    FunctionCommand *tempend = new FunctionCommand([](){
        drive_sys.stop();
        cata_sys.send_command(CataSys::Command::StopIntake);
        while(true)
        {
            cata_sys.send_command(CataSys::Command::StopIntake);
            double f = con.Axis3.position() / 200.0;
            double s = con.Axis1.position() / 200.0;
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
            pose_t pos = odom.get_position();
            printf("X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            vexDelay(100);
        }
        return false;
    });

    CommandController cmd{
        odom.SetPositionCmd({.x=18, .y=126, .rot=135}),
        
        // 1 - Turn and shoot preload
        // drive_sys.TurnToHeadingCmd(160, .5),
        cata_sys.Fire(),
        new DelayCommand(800),

        // 2 - Turn to matchload zone & begin matchloading
        drive_sys.TurnToHeadingCmd(135, .5),
        cata_sys.IntakeFully(),
        drive_sys.DriveToPointCmd({.x=13.8, .y=130.3}, FWD, 0.5),

        cata_sys.IntakeFully(),
        // Matchloading phase
        (new RepeatUntil(InOrder{
            intakeToCata->withTimeout(3),
            cata_sys.Fire(),
            // new Async{
            //     new InOrder{
            //         new DelayCommand(300),
            //         cata_sys.Fire(),
            //     }
            // },
            // drive_sys.DriveToPointCmd({.x=18, .y=126}, REV, 0.5)->withTimeout(1),
            drive_sys.DriveForwardCmd(8, REV, 0.5)->withTimeout(1),
            // drive_sys.TurnToHeadingCmd(160, 0.5)->withTimeout(1),
            // cata_sys.Fire(),
            // new DelayCommand(300),
            // drive_sys.TurnToHeadingCmd(135, 130.3)->withTimeout(1),
            cata_sys.IntakeFully(),
            drive_sys.DriveForwardCmd(8, FWD, 0.5)->withTimeout(1),
            
        }, new FunctionCondition([](){return false;})))
            ->withTimeout(45),

        // Last preload
        drive_sys.DriveToPointCmd({.x=18, .y=126}, FWD, 0.5)->withTimeout(1),
        drive_sys.TurnToHeadingCmd(160, 0.5)->withTimeout(1),
        cata_sys.Fire(),
        new DelayCommand(300),
        cata_sys.StopIntake(),
        drive_sys.TurnToHeadingCmd(204, 0.5)->withTimeout(1),

        // Drive through "The Passage" & push into side of goal
        // Push into side of goal
        // new Parallel{
        //     new InOrder{
        //         new WaitUntilCondition(new FunctionCondition([](){
        //             return odom.get_position().x > 100;
        //         })),
        //         new WingCmd(RIGHT, true)
        //     },
        cata_sys.StopIntake(),
        drive_sys.PurePursuitCmd(PurePursuit::Path({
            {.x=19, .y=133},
            {.x=40, .y=136},
            {.x=92, .y=136},
            {.x=108, .y=128},
            {.x=119, .y=117},
            {.x=123, .y=98},
            }, 8), REV, .5)->withTimeout(4),
        // },

        // FULL SPEED AHEAD
        drive_sys.DriveForwardCmd(18, FWD, 0.5)->withTimeout(1),
        drive_sys.TurnToHeadingCmd(150, 0.5),
        new WingCmd(RIGHT, true),
        drive_sys.TurnToHeadingCmd(90, 0.5),
        drive_sys.DriveForwardCmd(48, REV, 0.8)->withTimeout(1),
        
        // AGAIN!
        drive_sys.DriveForwardCmd(18, FWD, 0.5)->withTimeout(1),
        // drive_sys.DriveForwardCmd(48, REV, 0.8)->withTimeout(1),

        // // AGAIN!!!!
        // drive_sys.DriveForwardCmd(18, FWD, 0.5)->withTimeout(1),
        // drive_sys.DriveForwardCmd(48, REV, 0.8)->withTimeout(1),

        // drive_sys.PurePursuitCmd(PurePursuit::Path({
        //     {.x=0, .y=0},
        // }, 8), FWD, 0.3),

        //Wall Align
        drive_sys.TurnToHeadingCmd(90, 0.5)->withTimeout(1),
        drive_sys.DriveForwardCmd(100, FWD, 0.25)->withTimeout(3),
        cata_sys.Unintake(),
        drive_sys.TurnDegreesCmd(90, .5)->withTimeout(1),
        drive_sys.TurnDegreesCmd(-90, .5)->withTimeout(1),
        drive_sys.DriveForwardCmd(100, FWD, 0.25)->withTimeout(2),
        cata_sys.StopIntake(),

        odom.SetPositionCmd({138, 138, 45}),
        drive_sys.DriveToPointCmd({.x=127, .y=127}, REV, .5),
        new WingCmd(LEFT, true),

        // Curve to final position
        drive_sys.PurePursuitCmd(PurePursuit::Path({
            {.x=127, .y=127},
            {.x=118, .y=113},
            {.x=127, .y=98},
        }, 12.0), REV, 0.4),
        drive_sys.TurnToHeadingCmd(180, .5)->withTimeout(2),

        //  FULL SPEED AHEAD
        drive_sys.DriveForwardCmd(18, FWD, 0.5),
        drive_sys.DriveForwardCmd(48, REV, 0.8)->withTimeout(1),

        // Get out of the way & end
        new WingCmd(LEFT, false),
        drive_sys.TurnToHeadingCmd(129, .5),
        drive_sys.DriveForwardCmd(24, FWD, 0.5),
    };

    cmd.run();

    drive_sys.stop();
}