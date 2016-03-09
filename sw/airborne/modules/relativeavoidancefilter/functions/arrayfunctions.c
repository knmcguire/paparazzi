#include "arrayfunctions.h"

/*
	Shifts an array to the left by a defined number of steps at a time
*/
void array_shiftleft(float *array, int size, int shift)
{
	int i;
	float temp[size];
	memcpy(temp, array, size * sizeof(float));

	for (i = 0; i < size; i++)
	{
		if (i+shift >= size)
		{
			array[i] = temp[i+shift - size];
		} 
		else
		{
			array[i] = temp[i+shift];
		}

	}

}


/*
	Shifts an array to the right by a defined number of steps at a time
*/
void array_shiftright(float *array, int size, int shift)
{
	int i;
	float temp[size];
	memcpy(temp, array, size * sizeof(float));

	for (i = 0; i < size; i++)
	{
		if (i-shift < 0)
		{
			array[i] = temp[i-shift + size];
		} 
		else
		{
			array[i] = temp[i-shift];
		}

	}

}


/*
	Returns the idx of the minimum value in an array
*/
int array_getminidx(int length, float *x)
{
	int i;
	int idxmin = 0;
	float min = x[0];

	for (i = 1; i < length; i++)
	{
		if (x[i] < min)
		{
			min = x[i];
			idxmin = i;
		}
	}

	return idxmin;

}

/* 
	Returns the idx of the maximum value in an array
*/
int array_getmaxidx(int length, float *x)
{
	int i;
	int idxmax = 0;
	float max = x[0];

	for (i = 1; i < length; i++)
	{
		if (x[i] > max)
		{
			max = x[i];
			idxmax = i;
		}
	}

	return idxmax;
	
}


/* 

	Makes the vector x1 into a vector that features the minimum value
	at each index position when comparing vector x1 to vector x2 .

	x1 is altered.
	x2 is not altered.

*/
void array_arraymin(int length, float *x1, float *x2)
{
	int i;

	for (i = 0; i < length; i++)
	{
		if (x1[i] > x2[i])
		{
			x1[i] = x2[i];
		}
	}

}