#pragma once
#include "core.h"
#include <functional>

// ================ Autonomous Abstractions ================
class VisionTrackTriballCommand: public AutoCommand
{
    public:
    VisionTrackTriballCommand();
    bool run() override;

    private:
    PIDFF angle_fb;

}; 

// ================ Driver Assist Automations ================
void matchload_1(bool &enable);
void matchload_1(std::function<bool()> enable);
