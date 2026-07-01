#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>

void BeginSensor();
uint16_t getRawAngle();
float getAngle();
float getAngle_Without_track();

#endif
