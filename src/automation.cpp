#include "automation.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// ================ Driver Assist Automations ================

void matchload_1(bool &enable)
{
    matchload_1([enable](){return enable;});
}

void matchload_1(std::function<bool()> enable)
{
    if(!enable())
        return;
        
    FunctionCommand *waitForIntake = new FunctionCommand([&](){
        return intake_watcher.objectDistance(distanceUnits::mm) < 150 || cata_watcher.isNearObject();
    });

    FunctionCommand *waitForCata = new FunctionCommand([&](){

        return cata_watcher.isNearObject();
    });

    static timer drive_tmr;
    drive_tmr.reset();
    CommandController cmd{
        cata_sys.IntakeFully(),
        new FunctionCommand([](){
            drive_sys.drive_tank(0.2, 0.2);
            return drive_tmr.time(sec) > .4;
        }),
        waitForCata, 
        drive_sys.DriveForwardCmd(12, REV, .5),
        drive_sys.TurnDegreesCmd(20, .5),
        cata_sys.Fire(), 
        new DelayCommand(500),
        cata_sys.IntakeFully(),
        drive_sys.TurnDegreesCmd(-20, .5),
        drive_sys.DriveForwardCmd(12, FWD, .5),
    };

    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&](){return !enable(); });
    cmd.run();
    cata_sys.send_command(CataSys::Command::StopIntake);
}