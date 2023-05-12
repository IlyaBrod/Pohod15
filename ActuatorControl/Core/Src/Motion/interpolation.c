#include "interpolation.h"
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx_it.h"

void interpol_init(interpol * data, float * ptx, float * pty, uint16_t size) {

	float * hi;
	float * eti;
	MAT *A, *LU;
	VEC *B;
	PERM *pivot;

	//Setting the cubic spline system to find interpolation
	//coefficients
	hi = (float *)(calloc(size,sizeof(float)));
	eti = (float *)(calloc(size,sizeof(float)));
	if(hi==NULL || eti==NULL) {
			MemManage_Handler();
	}

	for(int i=0;i<size-2;i++) {
		hi[i] = ptx[i+1]-ptx[i];
		eti[i] = pty[i+1]-pty[i];
	}

	A = m_get(size,size);
	B = v_get(size);

	for(int i=0;i<size-3;i++) {
		A->me[i+1][i]   = 1/3*hi[i];
		A->me[i+1][i+1] = 2/3*(hi[i]+hi[i+1]);
		A->me[i+1][i+2] = 1/3*hi[i+1];
		B->ve[i+1] = eti[i+1]/hi[i+1] - eti[i]/hi[i];
	}

	//not-a-knot condition
	A->me[0][0] = hi[1];
	A->me[0][1] = -(hi[0]+hi[1]);
	A->me[0][2] = hi[0];
	A->me[size-1][size-3] = hi[size-2];
	A->me[size-1][size-2] = -(hi[size-3]+hi[size-2]);
	A->me[size-1][size-1] = hi[size-3];

	LU = m_get(A->m,A->n);
	LU = m_copy(A,LU);
	pivot = px_get(A->m);
	LUfactor(LU,pivot);
	data->bi = LUsolve(LU,pivot,B,VNULL);

	data-> ai = v_get(size-1);
	data-> ci = v_get(size-1);
	data-> di = v_get(size-1);

	for(int i=1;i<size-2;i++){
		data->ai->ve[i] = 1/(3*hi[i])*((data->bi->ve[i+1])-(data->bi->ve[i]));
		data->ci-> ve[i] = eti[i]/hi[i] - hi[i]/3*((data->bi->ve[i+1])+2*(data->bi->ve[i]));
		data->di->ve[i] = pty[i];
	}

	V_FREE(B);
	M_FREE(A);
	M_FREE(LU);
	PX_FREE(pivot);
	free(hi);
	free(eti);

}


float interpol_get(float x, interpol * data, float * ptx, uint16_t size) {

	for(int i=1;i<size;i++) {
		if(x<ptx[0]) {
			 x = ptx[size-1]-(ptx[0]-x);
		} else if(x>ptx[size-1]) {
			 x = ptx[0] + x-ptx[size-1];
		}

		for(int j=0;j<size-2;j++) {
			if(x<=ptx[i+1]) {
				return (data->ai->ve[i])*pow(x-ptx[i],3)+(data->bi->ve[i])*pow(x-ptx[i],2)+(data->ci->ve[i])*(x-ptx[i])+(data->di->ve[i]);
			}
		}
	}

	return 0;
}
