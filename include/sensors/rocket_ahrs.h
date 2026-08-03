#pragma once
#include "Fusion.h"

bool setup_fusion();
void update_fusion(float ax, float ay, float az, 
                   float gx, float gy, float gz, 
                   float dt);
float get_vertical_acceleration_ms2();
FusionAhrs* ahrs_get();
void calibrate_vertical_bias(int samples = 200, void (*tick)() = nullptr);