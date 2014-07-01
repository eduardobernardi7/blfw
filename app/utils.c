#include "utils.h"

int16_t circindex(int16_t index, int16_t incr, int16_t nitems) {
	index += incr;
	if (index < 0) {
		index += nitems;
	} else {
		if (index >= nitems) {
			index -= nitems;
		}
	}
	return index;
}

void sort(uint32_t * buf, uint16_t size)
{
	uint16_t i,j;
	uint32_t hold;

	for(i = 1; i < size; i++)
	{
		j = i;
		while(buf[j] < buf[j-1])
		{
			hold = buf[j];
			buf[j] = buf[j-1];
			buf[j-1] = hold;
			j--;
			if (j == 0) break;
		}
	}
}

/* TODO: Reescrever para utilizar Minimos quadrados */
void reg_lin(float v[2], float  r[2], float * coef_ang, float * coef_lin)
{
	int32_t i,j,k;
	float mult,s;
	uint16_t n = 2;
	float a[2][2];
	float b[2];
	float x[2];

	a[0][0] = v[0];
	a[0][1] = 1;
	a[1][0] = v[1];
	a[1][1] = 1;

	b[0] = r[0];
	b[1] = r[1];

	for(k = 0; k < n-1; k++)
	{
		for (i = k+1; i < n ; i++)
		{
			mult = a[i][k] / a[k][k];
			a[i][k] = 0;
			for(j = k; j < n; j++)
			{
				a[i][j] = mult*a[k][j] - a[i][j];
			}
			b[i] = mult*b[k] - b[i];
		}
	}

	x[n-1] = b[n-1]/a[n-1][n-1];
	for(k = n-2 ; k >= 0; k--)
	{
		s = 0;
		for(j = k+1; j < n; j++)
		{
			s = s + a[k][j]*x[j];
		}
		x[k] = (b[k] - s)/a[k][k];
	}

	*coef_ang = x[0];
	*coef_lin = x[1];

}

void gauss(float ** a, float * b, float * x, uint16_t n)
{
	int32_t i,j,k;
	float mult,s;


	for(k = 0; k < n-1; k++)
	{
		for (i = k+1; i < n ; i++)
		{
			mult = a[i][k] / a[k][k];
			a[i][k] = 0;
			for(j = k; j < n; j++)
			{
				a[i][j] = mult*a[k][j] - a[i][j];
			}
			b[i] = mult*b[k] - b[i];
		}
	}

	x[n-1] = b[n-1]/a[n-1][n-1];
	for(k = n-2 ; k >= 0; k--)
	{
		s = 0;
		for(j = k+1; j < n; j++)
		{
			s = s + a[k][j]*x[j];
		}
		x[k] = (b[k] - s)/a[k][k];
	}
}
