#include "automation.h"
#include "robot-config.h"
#include "vision.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// VISION TRACKING
vision_filter_s default_vision_filter = {
    .min_area = 500,
    .aspect_low = 0.8,
    .aspect_high = 1.2,

    .min_x = 0,
    .max_x = 320,
    .min_y = 0,
    .max_y = 240,
};

FeedForward::ff_config_t angle_ff_cfg {

};
PID::pid_config_t angle_pid_cfg {

};

VisionTrackTriballCommand::VisionTrackTriballCommand(vision_filter_s &filter)
: angle_fb(angle_pid_cfg, angle_ff_cfg), filter(filter)
{}

bool VisionTrackTriballCommand::run()
{
    static const int center_x = 160;
    static const double max_drive_speed = 0;// 0.3;
    static const double max_angle_speed = 0.3;
    static const double area_speed_scalar = 1.0 / 500.0; // Drive pct over area

    std::vector<vision::object> sensed_obj = vision_run_filter(TRIBALL);

    if(sensed_obj.size() <= 0)
    {
        // Stop & wait if there isn't anything sensed
        drive_sys.stop();
        return false;
    }

    if(intake_watcher.objectDistance(distanceUnits::mm) < 100)
    {
        // Done when triball is in the intake
        drive_sys.stop();
        return true;
    }

    // Get the largest object sensed
    vision::object largest;
    for(vision::object &obj : sensed_obj)
    {
        if((obj.width * obj.height) > (largest.width * largest.height))
            largest = obj;
    }

    double object_area = largest.width * largest.height;

    angle_fb.set_target(center_x);
    angle_fb.update(largest.centerX);
    angle_fb.set_limits(-max_angle_speed, max_angle_speed);
    
    // Slow down as size of object increases (big area = small speed)
    // TODO test this
    // double speed = clamp(1-(area_speed_scalar * object_area), 0, max_drive_speed);
    double speed = max_drive_speed;

    drive_sys.drive_tank_raw(speed - angle_fb.get(), speed + angle_fb.get());

    return false;
}

std::vector<vision::object> vision_run_filter(vision::signature &sig, vision_filter_s &filter)
{
    cam.takeSnapshot(TRIBALL);
    vision::object &sensed = cam.objects[0];
    std::vector<vision::object> out;

    // Go through all sensed objects
    for(int i = 0; i < cam.objectCount; i++)
    {
        vision::object &cur_obj = cam.objects[i];

        // Filtering by size, aspect ratio & location in frame
        int area = cur_obj.width * cur_obj.height;
        double aspect_ratio = cur_obj.width / cur_obj.height;
        int x = cur_obj.centerX;
        int y = cur_obj.centerY;

        // keep searching if filtered
        if (area < filter.min_area
            || aspect_ratio < filter.aspect_low || aspect_ratio > filter.aspect_high
            || x < filter.min_x || x > filter.max_x || y < filter.min_y || y > filter.max_y)
        {
            continue;
        }

        out.push_back(cur_obj);
    }
    
    return out;
}

VisionObjectExists::VisionObjectExists(vision_filter_s &filter)
: filter(filter)
{}

bool VisionObjectExists::test()
{
    return vision_run_filter(TRIBALL, this->filter).size() > 0;
}

// ================ Driver Assist Automations ================

void matchload_1(bool &enable)
{
    matchload_1([enable](){return enable;});
}

void matchload_1(std::function<bool()> enable)
{
    if(!enable())
        return;

    FunctionCommand *intakeToCata = new FunctionCommand([](){
        drive_sys.drive_tank(0.15, 0.15);
        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    static timer drive_tmr;
    drive_tmr.reset();
    CommandController cmd{
        cata_sys.IntakeFully(),
        intakeToCata->withTimeout(3),
        new Async{
            new InOrder{
                new DelayCommand(200),
                cata_sys.Fire(),
            }
        },
        drive_sys.DriveForwardCmd(14, REV, 0.6)->withTimeout(1),
        new FunctionCommand([](){cata_sys.send_command(CataSys::Command::StopFiring); return true;}),
        cata_sys.IntakeFully(),
        drive_sys.DriveForwardCmd(14, FWD, 0.6)->withTimeout(1),
    };

    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&](){return !enable(); });
    cmd.run();
    cata_sys.send_command(CataSys::Command::StopIntake);
}