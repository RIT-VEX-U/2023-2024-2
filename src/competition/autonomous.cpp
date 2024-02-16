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

FunctionCondition *is_in_cata()
{
    return new FunctionCondition([](){
        return cata_watcher.isNearObject();
    });
}

void skills()
{
    auto tempend = new DebugCommand();

    CommandController cmd{
        odom.SetPositionCmd({.x=19, .y=117, .rot=135}), // GPS says starting pt is 
        (new RepeatUntil(InOrder{

            // Matchloading!
            cata_sys.IntakeFully(),
            // Push against bar slowly & wait for triball to load
            new FunctionCommand([](){
                drive_sys.drive_tank(0.3, 0.3);
                return true;
            }),

            // Up against thte wall, reset odometry
            cata_sys.WaitForIntake()->withTimeout(2),
            odom.SetPositionCmd({.x=15, .y=121, .rot = 135}),

            new FunctionCommand([](){
                vex::task([](){
                    vexDelay(600);
                    cata_sys.send_command(CataSys::Command::StartFiring);
                    return 0;
                });
                
                return true;
            }),

            // Drive to firing position
            drive_sys.PurePursuitCmd(drive_pid, PurePursuit::Path({
                {.x=15, .y=121},
                {.x=17, .y=120},
                {.x=19, .y=120},
                {.x=27, .y=120},
            }, 4), REV, 0.3),
            
            drive_sys.DriveForwardCmd(8, FWD)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(135)->withTimeout(3),
            drive_sys.DriveForwardCmd(6, FWD)->withTimeout(3),
                        
            // drive_sys.PurePursuitCmd(drive_pid, PurePursuit::Path({
            //     {.x=27, .y=120},
            //     {.x=23, .y=120},
            //     {.x=20, .y=120},
            //     {.x=16, .y=125},
            // }, 4), FWD, 0.3)->withTimeout(1),

            // drive_sys.DriveForwardCmd(8, REV),
            // drive_sys.TurnToHeadingCmd(200),
            // drive_sys.DriveForwardCmd(6, REV),
            // drive_sys.DriveForwardCmd(6, FWD),
            // drive_sys.TurnToHeadingCmd(135),
            // drive_sys.DriveForwardCmd(8, FWD),
            // May need a delay here, we'll see
            // Start intake & drive back to loading area
            cata_sys.IntakeFully(),
           
            // Done, go back to beginning of matchloading
            
        }, new FunctionCondition([](){ return false; })))->withTimeout(45),
        // If there is a ball left in the catapult, fire it & THEN push
        // new Branch{
        //     is_in_cata(),
        //     new InOrder { // FALSE
        //         // do nothing
        //     },
        //     // Copied from above TRUE
        //     new Async(new InOrder{
        //         new WaitUntilCondition(new FunctionCondition([](){
        //             return odom.get_position().y > 0;
        //         })),
        //         cata_sys.Fire()
        //     })
        // },

        // Deploy wing while driving after crossing X value
        // May not be needed for side goal
        new Async(new InOrder{
            new WaitUntilCondition(new FunctionCondition([](){
                return odom.get_position().x > 90;
            })),
            new WingCmd(RIGHT, true),
             new WingCmd(LEFT, true),
            new WaitUntilCondition(new FunctionCondition([](){
                return odom.get_position().x > 125;
            })),
            new WingCmd(RIGHT, false),
            new WingCmd(LEFT, false),
        }),

        // Drive SLOWLY under bar (don't push just yet)
        drive_sys.PurePursuitCmd(drive_pid, PurePursuit::Path({
            {.x=15, .y=122},
            {.x=21, .y=119},
            {.x=29, .y=121},
            {.x=41, .y=125},
            {.x=71, .y=127},
            {.x=98, .y=126},
            {.x=117, .y=120},
            {.x=129, .y=110},
        }, 8), REV, 0.4),
        new WingCmd(RIGHT, false),
        
        drive_sys.TurnToHeadingCmd(90),

        // Ram, back up & ram again
        drive_sys.DriveForwardCmd(drive_pid, 24, REV, 0.8)->withTimeout(1),
        drive_sys.DriveForwardCmd(18, FWD),
        drive_sys.TurnToHeadingCmd(100),
        drive_sys.DriveForwardCmd(drive_pid, 24, REV, 0.8)->withTimeout(1),
        drive_sys.DriveForwardCmd(12, FWD),

        tempend,

        // Backup & localize
        drive_sys.PurePursuitCmd(drive_pid, PurePursuit::Path({
            {.x=0, .y=0},
            {.x=0, .y=0},
            {.x=0, .y=0},
        }, 8), FWD, 0.4),
        drive_sys.TurnToHeadingCmd(45),
        new GPSLocalizeCommand(RED),

    };

    cmd.run();

    drive_sys.stop();
}