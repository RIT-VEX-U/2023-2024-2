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

}

void skills()
{
    FunctionCommand *intakeToCata = new FunctionCommand([](){
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

    CommandController cmd{
        odom.SetPositionCmd({.x=0, .y=0, .rot=0}),
        
        // 1 - Turn and shoot preload
        drive_sys.TurnToHeadingCmd(0, .5),
        cata_sys.Fire(),
        new DelayCommand(300),

        // 2 - Turn to matchload zone & begin matchloading
        drive_sys.TurnToHeadingCmd(0, .5),
        cata_sys.IntakeFully(),
        drive_sys.DriveToPointCmd({.x=0, .y=0}, FWD, 0.5),

        // Matchloading phase
        new RepeatUntil(InOrder{
            intakeToCata,
            drive_sys.DriveToPointCmd({.x=0, .y=0}, REV, 0.5),
            drive_sys.TurnToHeadingCmd(0, 0.5),
            cata_sys.Fire(),
            new DelayCommand(300),
            drive_sys.TurnToHeadingCmd(0, 0.5),
            cata_sys.IntakeFully(),
            drive_sys.DriveToPointCmd({.x=0, .y=0}, FWD, 0.5)
        }, new IfTimePassed(30)),

        // Last preload
        cata_sys.Fire(),
        new DelayCommand(300),
        drive_sys.TurnToHeadingCmd(0, 0.5),

        // Drive through "The Passage" & push into side of goal
        // Push into side of goal
        drive_sys.TurnToHeadingCmd(0, 0.5),
        new Parallel{
            new InOrder{
                new WaitUntilCondition(new FunctionCondition([](){
                    return odom.get_position().y > 0;
                })),
                new WingCmd(LEFT, true)
            },
            drive_sys.PurePursuitCmd(PurePursuit::Path({
                {.x=0, .y=0},
                {.x=0, .y=0},
                {.x=0, .y=0},
                }, 12), REV, .5)
        },

        // Undeploy wing and back up, deploy other wing
        new WingCmd(LEFT, false),
        drive_sys.DriveToPointCmd({.x=0, .y=0}, FWD, 0.5),
        drive_sys.TurnToHeadingCmd(0, 0.5),
        new WingCmd(RIGHT, true),

        // Pure pursuit to center goal
        drive_sys.PurePursuitCmd(PurePursuit::Path({
            {.x=0, .y=0},
            {.x=0, .y=0},
            {.x=0, .y=0},
        }, 12.0), REV, 0.5),
        drive_sys.DriveToPointCmd({.x=0, .y=0}, FWD, 0.5),
        new WingCmd(RIGHT, false)
    };

    cmd.run();

    drive_sys.stop();
}