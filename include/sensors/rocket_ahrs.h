#pragma once

bool setup_fusion();
void update_fusion(float ax, float ay, float az, 
                   float gx, float gy, float gz, 
                   float dt);
float get_true_vertical_acceleration();