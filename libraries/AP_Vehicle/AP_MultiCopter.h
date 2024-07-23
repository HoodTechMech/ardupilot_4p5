#pragma once

#include <AP_Param/AP_Param.h>

/*
  common parameters for multicopters
*/
struct AP_MultiCopter {
    AP_Int16 angle_max;
    AP_Int16 dash_angle_cdeg;
};
