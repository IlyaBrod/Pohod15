#include "interpolation.h"
#include "motion.h"
#include "kinematic.h"
#include "LEG_CONFIG.h"
#include "customVars.h"

interpol splineZ;
interpol splineY;

#define PTS_NUMBER_Z 8
#define PTS_NUMBER_Y 7
float ptx_z[PTS_NUMBER_Z] = {0, 0.1, 0.2, 0.4, 0.5, 0.7, 0.9, 1}; //Z timings
float pty_z[PTS_NUMBER_Z] = {1, 1.14, 1, 1, 0, 0, 0, 1};		   //Z values
float ptx_y[PTS_NUMBER_Y] = {0, 0.1, 0.25, 0.5, 0.75, 0.9, 1};	   //Y timings
float pty_y[PTS_NUMBER_Y] = {0, 0.14, 0.6, 1, 0.6, 0.14, 0};	   //Y values

float highOffset = DEFAULT_STEP_HIGH_OFFSET;
float high = DEFAULT_STEP_HIGH;
float dist = DEFAULT_STEP_DIST;
float step = DEFAULT_STEP_LENGTH;

float prev_output[3] = {0,0,0};

void motion_init()
{
	interpol_init(&splineZ, ptx_z,pty_z, PTS_NUMBER_Z);
	interpol_init(&splineY, ptx_y,pty_y, PTS_NUMBER_Y);
}

void motion_get(float t, float output[3])
{

	int Z = (int)(interpol_get(t, &splineZ,ptx_z,PTS_NUMBER_Z))*high+highOffset;
	int Y = (int)(interpol_get(t, &splineY,ptx_y,PTS_NUMBER_Y))*step-step/2;
	int X = (int)(dist);
	int input[3] = {X,Y,Z};

	bool state = computeIK(input, output);
	if(state==false) {
		output[0] = prev_output[0];
		output[1] = prev_output[1];
		output[2] = prev_output[2];
	}
}

void motion_set_h(float h_)
{
    high = h_;
}

void motion_set_hoffset(float h_)
{
    highOffset = h_;
}


void motion_set_d(float d_)
{
    dist = d_;
}

