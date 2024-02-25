#pragma once
#include "core.h"
#include <functional>
#include <tuple>

// ================ Autonomous Abstractions ================
struct vision_filter_s {
  int min_area;
  int max_area;

  double aspect_low;
  double aspect_high;

  int min_x;
  int max_x;
  int min_y;
  int max_y;
};

extern vision_filter_s default_vision_filter;

class VisionTrackTriballCommand : public AutoCommand {
public:
  VisionTrackTriballCommand(vision_filter_s &filter = default_vision_filter);
  bool run() override;

private:
  PIDFF angle_fb;
  vision_filter_s &filter;
};

class VisionObjectExists : public Condition {
public:
  VisionObjectExists(vision_filter_s filter = default_vision_filter);
  bool test() override;

private:
  vision_filter_s filter;
};

std::vector<vision::object> vision_run_filter(vision::signature &sig, vision_filter_s filter = default_vision_filter);

point_t estimate_triball_pos(vision::object &obj);

class IsTriballInArea : public Condition {
public:
  IsTriballInArea(point_t pos, int radius, vision_filter_s &filter = default_vision_filter);
  bool test() override;

private:
  vision_filter_s &filter;
  point_t pos;
  int radius;
};

void gps_localize_median();
std::tuple<pose_t, double> gps_localize_stdev();

enum FieldSide { RED, BLUE };

class GPSLocalizeCommand : public AutoCommand {
public:
  GPSLocalizeCommand(FieldSide s);
  bool run() override;
  static pose_t get_pose_rotated();

private:
  FieldSide side;
  static bool first_run;
  static int rotation;
  static const int min_rotation_radius;
};

// ================ Driver Assist Automations ================
void matchload_1(bool &enable);
void matchload_1(std::function<bool()> enable);
