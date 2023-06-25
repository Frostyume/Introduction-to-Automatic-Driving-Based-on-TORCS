#include "tgf.h"

#ifndef __USER_ITF
#define __USER_ITF

/* CyberCruise User Interface */
typedef void (*tfudGetParam) (float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
typedef void (*tfudSetParam) (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);

typedef  struct {
	tfudGetParam userDriverGetParam;
	tfudSetParam userDriverSetParam;
} tUserItf;

#endif