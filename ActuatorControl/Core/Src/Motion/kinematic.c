
#include "kinematic.h"
#include "LEG_CONFIG.h"
#include <math.h>

/**
 * Compute the kinematics of the robot
 */
bool computeIK(float input[3], float output[3])
{
	float X = input[0];
	float Y = input[1];
	float Z = input[2];
	output[2] = (float)(atan2(Y,(X-L_TORAX)));

	float Xp = (float)sqrt(pow((X-L_TORAX),2)+pow(Y,2));
	float L = sqrt(pow(Z,2)+pow(Xp-L_COXA,2));
	float beta = PI/2 - atan2(Xp-L_COXA,Z);
	output[1] = acos((pow(L_FEMUR,2)+pow(L,2)-pow(L_TIBIA,2))/(2*L_FEMUR*L))-beta;
	output[0] = acos((Xp-L_COXA-L_FEMUR*cos(output[1]))/L_TIBIA) + output[1];

	return true;
}
