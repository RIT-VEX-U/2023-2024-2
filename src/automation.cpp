#include "automation.h"
#include "robot-config.h"
#include "vision.h"
#include "../core/include/utils/geometry.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// VISION TRACKING
vision_filter_s default_vision_filter = {
    .min_area = 300,
    .max_area = 100000,
    .aspect_low = 0.8,
    .aspect_high = 4,

    .min_x = 0,
    .max_x = 320,
    .min_y = 0,
    .max_y = 240,
};

FeedForward::ff_config_t angle_ff_cfg {

};
PID::pid_config_t angle_pid_cfg {
    .p = 0.004,
    .d = 0.0005
};

VisionTrackTriballCommand::VisionTrackTriballCommand(vision_filter_s &filter)
: angle_fb(angle_pid_cfg, angle_ff_cfg), filter(filter)
{}

bool VisionTrackTriballCommand::run()
{
    static const int center_x = 160;
    static const double min_drive_speed = 0.1;
    static const double max_drive_speed = 0.3;
    
    static const double max_angle_speed = 0.3;
    static const double area_speed_scalar = 50000; // Area at which speed is zero

    vision_light.set(true);
    std::vector<vision::object> sensed_obj = vision_run_filter(TRIBALL);

    if(intake_watcher.objectDistance(distanceUnits::mm) < 100)
    {
        // Done when triball is in the intake
        drive_sys.stop();
        vision_light.set(false);
        return true;
    }

    if(sensed_obj.size() <= 0)
    {
        // Stop & wait if there isn't anything sensed
        drive_sys.stop();
        return false;
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
    double speed = clamp(lerp(1, 0, object_area / area_speed_scalar), 0, 1) * max_drive_speed + min_drive_speed;
    // double speed = max_drive_speed;
    printf("x: %d\n", largest.centerX);
    drive_sys.drive_tank_raw(speed + angle_fb.get(), speed - angle_fb.get());

    return false;
}

std::vector<vision::object> vision_run_filter(vision::signature &sig, vision_filter_s filter)
{
    cam.takeSnapshot(sig);
    std::vector<vision::object> out;

    // Go through all sensed objects
    for(int i = 0; i < cam.objectCount; i++)
    {
        vision::object &cur_obj = cam.objects[i];

        // Filtering by size, aspect ratio & location in frame
        int area = cur_obj.width * cur_obj.height;
        double aspect_ratio = ((double)cur_obj.width) / ((double)cur_obj.height);
        int x = cur_obj.centerX;
        int y = cur_obj.centerY;
        // printf("areaMin %d %d %d\n", area < filter.min_area, area, filter.min_area);
        // printf("areaMax %d %d\n", area > filter.max_area, filter.max_area);
        // printf("ARMin %d %f %f\n", aspect_ratio < filter.aspect_low, aspect_ratio, filter.aspect_low);
        // printf("ARMax %d %f\n", aspect_ratio > filter.aspect_high, filter.aspect_high);
        // printf("minx %d %d %d\n", x < filter.min_x, x, filter.min_x);
        // printf("maxx %d %d\n", x > filter.max_x, filter.max_x);
        // printf("miny %d %d %d\n", y < filter.min_y, y, filter.min_y);
        // printf("maxy %d %d\n", y > filter.max_y, filter.max_y);


        // keep searching if filtered
        if (area < filter.min_area || area > filter.max_area
            || aspect_ratio < filter.aspect_low || aspect_ratio > filter.aspect_high
            || x < filter.min_x || x > filter.max_x || y < filter.min_y || y > filter.max_y)
        {
            continue;
        }

        out.push_back(cur_obj);
    }

    // Only for debugging
    // printf("Num: %d\n", out.size());
    
    return out;
}

VisionObjectExists::VisionObjectExists(vision_filter_s filter)
: filter(filter)
{}

bool VisionObjectExists::test()
{
    return vision_run_filter(TRIBALL, this->filter).size() > 0;
}

point_t estimate_triball_pos(vision::object &obj)
{
    pose_t robot_pose = odom.get_position();

    double area = obj.width * obj.height;
    double dist = 3721*pow(area, -0.634); // Estimate found by spreadsheet
    double heading = 53.9 + (0.241*obj.centerX) + (0.0000305 * area*area); // Estimate found by spreadsheet
    Vector2D object_vec(deg2rad(heading), dist);

    // In reference to the camera
    point_t local_pos = object_vec.point();
    printf("locX: %f, locY: %f, ", local_pos.x, local_pos.y);

    // Rotate in reference to the robot's heading
    point_t field_pos = (Mat2::FromRotationDegrees(robot_pose.rot) * local_pos) + robot_pose.get_point();
    printf("absX: %f, absY: %f\n", field_pos.x, field_pos.y);
    return field_pos;
}

IsTriballInArea::IsTriballInArea(point_t pos, int radius, vision_filter_s &filter)
: filter(filter), pos(pos), radius(radius) {}

bool IsTriballInArea::test()
{
    std::vector<vision::object> objs = vision_run_filter(TRIBALL, filter);

    // Run through all objects sensed & return true if any is within a certain distance
    for (int i=0; i<objs.size(); i++)
    {
        point_t obj_pos = estimate_triball_pos(objs[i]);
        if (fabs(obj_pos.dist(pos)) <= radius)
            return true;
    }

    return false;
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