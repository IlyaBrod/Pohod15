#ifndef IK_H
#define IK_H

#include "customVars.h"

/**
 * Compute inverse kinematics of the 3Dof leg
 * @param input[3] = [X,Y,Z]
 * @param output[3] =  [AGL_J1,AGL_J2,AGL_J3]
 * @return false if error true else
 */
bool computeIK(float input[3], float output[3]);

#endif
