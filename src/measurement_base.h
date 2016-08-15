#ifndef MEASUREMENT_BASE_H
#define MEASUREMENT_BASE_H

#include "common_definitions.h"
#include "source.h"

#include "axis_angle.h"
#include "parameter_config.h"

#include "frame.h"
#include "key_frame.h"
#include "state_variables.h"

#include "icp.h"
#include "loop_closure.h"

#include "kalman_filter.h"

#include "visualizer.h"

class measurementBase
{
public:
    measurementBase();
};

#endif // MEASUREMENT_BASE_H
