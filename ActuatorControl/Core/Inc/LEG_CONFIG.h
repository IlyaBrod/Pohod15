/**
	*	Functions to control Pohod Legs
	*
	*
	*							 J1						O  o					J1'
	*							/   \				|	BODY |			/			\
	*						/				J2---J3			J3'---J2'				\
	*				   V 																		 V
	*/

#ifndef LEG_CONFIG
#define LEG_CONFIG

//#################################################		GENERAL
#define PI (float)(3.14159265359f)
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define RAD2DEG(a) ((a)*180.0/PI)
#define DEG2RAD(a) ((a)*PI/180.0)


//#################################################		DEBUG ON/OFF
//Uncomment to debug
//#define DEBUG_SPEED
#define DEBUG_ANGLE_DEG
//#define DEBUG_TECH
#define DEBUG_SAVE_IO

//#################################################		CONTROL DESIGN
#define DEFAULT_STEP_LENGTH 100 //[mm]
#define DEFAULT_STEP_HIGH 100 //[mm]
#define DEFAULT_STEP_HIGH_OFFSET 50 //[mm]
#define DEFAULT_STEP_DIST 330 //[mm] Distance from the leg fixation to the tip along x
#define DEFAULT_FREQ 1 //[Hz]

#define FILTER_NB 3//Control filter order (numerator)
#define FILTER_NA 3//Control filter order (denominator)
#define FILTER_N ((FILTER_NB <= FILTER_NA) ? FILTER_NA : FILTER_NB)

#define CMD_N 80 //Number of points in the command

#define ROBOT_VOLTAGE (double)(12.0) //Volts
#define VOLT_TO_PWM (double)((double)(PWM_PERIOD)/ROBOT_VOLTAGE)

//#################################################		JOINTS PARAMS

#define JOINT1
#define JOINT2
#define JOINT3

#define ID1	0
#define ID2	1
#define ID3	2

//Minimal and maximal angles definition [unit: sensor ticks]
#define ID1_AGL_MIN (uint16_t)865
#define ID1_AGL_MAX (uint16_t)1529
#define ID2_AGL_MIN (uint16_t)250
#define ID2_AGL_MAX (uint16_t)805
#define ID3_AGL_MIN (uint16_t)523
#define ID3_AGL_MAX (uint16_t)1049

#define ID1_DEFAULT_ANGLE (ID1_AGL_MAX - ID1_AGL_MIN)/2 //[] reset angle of each joint
#define ID2_DEFAULT_ANGLE (ID2_AGL_MAX - ID2_AGL_MIN)/2 //[]
#define ID3_DEFAULT_ANGLE (ID3_AGL_MAX - ID3_AGL_MIN)/2 //[]

//Clockwise - counter clockwise rotation setting [0;1]
#define ID1_POLARITY 1
#define ID2_POLARITY 0
#define ID3_POLARITY 0

#define ERROR_BEFORE_WAIT 0.017f //in rads
	
//Geometric DATA
//Distances between joints
#define L_TIBIA 206 //[mm]
#define L_FEMUR 117 //[mm]
#define L_COXA 27 //[mm]
#define L_TORAX 134 //[mm]

#define OFFSET_ANGLE (float)(36.0f) //Leg y axis angle offset




#endif
