#ifndef __KINEMATICS_H
#define __KINEMATICS_H	 
#include "sys.h"

float getCurAngle(void);

float getCurVelocity(void);

float getCurAngleVel(void);
	
int Angle2Position(float Angle);

int getExpPosition(float w,float v);

int getExpRightVelocity(float v);

int getExpLeftVelocity(float v);



#endif 













