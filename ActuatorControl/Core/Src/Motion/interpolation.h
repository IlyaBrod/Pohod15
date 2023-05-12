#ifndef INTERPOL_H
#define INTERPOL_H

#include "../Meschach/matrix2.h"

//Interpolation coefficients storage
typedef struct {
	VEC * ai;
	VEC * bi;
	VEC * ci;
	VEC * di;
}interpol;


/**
 * @brief Initialize cubic spline interpolation structure with the provided points
 * @param data interpolation structure pointer
 * @param ptx abscissa coordinates of provided points
 * @param pty ordinate coordinates of provided points
 * @param size number of points
 */
void interpol_init(interpol * data, float * ptx, float * pty, uint16_t size);

/**
 * @brief Cubic spline interpolation  between provided initalization points
 * @param x value for which the interpolation should be done
 * @param data interpolation structure pointer
 * @param ptx abscissa coordinates of provided points
 * @param size number of points
 */
float interpol_get(float x, interpol * data, float * ptx, uint16_t size);


#endif
