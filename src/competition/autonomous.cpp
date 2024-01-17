#include "competition/autonomous.h"
#include "robot-config.h"
#include "core.h"
#include <functional>

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

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
    while(imu.isCalibrating()){vexDelay(100);}
    skills();
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

        new FunctionCommand([](){cata_sys.send_command(CataSys::Command::StopFiring); return true;}),
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
            new FunctionCommand([](){cata_sys.send_command(CataSys::Command::StopFiring); return true;}),
            cata_sys.IntakeFully(),
            drive_sys.DriveForwardCmd(8, FWD, 0.5)->withTimeout(1),
            
        }, new FunctionCondition([](){return false;})))
            ->withTimeout(5),

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
        cata_sys.Outtake(),
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