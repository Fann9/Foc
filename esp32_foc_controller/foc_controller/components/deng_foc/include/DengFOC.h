#ifndef DENG_FOC_H
#define DENG_FOC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void setPwm(float Ua, float Ub, float Uc);
void setTorque(float Uq, float angle_el);
float _normalizeAngle(float angle);
void DFOC_Vbus(float power_supply);
void DFOC_alignSensor(int _PP, int _DIR);
float _electricalAngle();
float DFOC_M0_Angle();

#ifdef __cplusplus
}
#endif

#endif
