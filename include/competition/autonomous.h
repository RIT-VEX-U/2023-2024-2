#pragma once
#include "core.h"

FunctionCommand* gps_reset();

void scoreAutoAWP();
void scoreAutoFull();
void skills();


/**
 * Main entrypoint for the autonomous period
*/
void autonomous();