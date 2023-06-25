/***************************************************************************

    file                 : user1.h
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberCruise

 ***************************************************************************/

#include "tgf.h"
#include<cmath>

#ifndef __USER_ITF
#define __USER_ITF

/* CyberCruise User Interface */
typedef void (*tfudGetParam) (float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
typedef void (*tfudSetParam) (float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);

typedef  struct {
	tfudGetParam userDriverGetParam;
	tfudSetParam userDriverSetParam;
} tUserItf;

#endif