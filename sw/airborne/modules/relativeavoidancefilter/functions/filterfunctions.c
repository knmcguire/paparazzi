#include "filterfunctions.h"

void zero(float *matrix, int row, int col)
{
	int i,j;
	for(i = 0 ; i < row; i++)
	{
		for(j = 0 ; j < col; j++)
		{
			matrix[i*col+j] = 0.0;
		}
	}
};

void identity(float *matrix, int n)
{
	int i,j;
	for(i = 0 ; i < n; i++)
	{
		for(j = 0 ; j < n; j++)
		{
			if (i == j)
			{
				matrix[i*n+j] = 1.0;
			}
			else
			{
				matrix[i*n+j] = 0.0;
			}
		}
	}
};

void linear_filter(float *u, float* X, float* dt, float *dX, float* A)
{
	UNUSEDVAR(*u);

	*dt = 0.2;

	/* dX */
	// Make a zero vector
	zero(dX,9,1);
	dX[0] = -(X[2] - X[4])*(*dt);
	dX[1] = -(X[3] - X[5])*(*dt);
	
	/* F'(x) */
	// make an identity matrix
	identity(A,9);
	A[0*9+2] = -*dt;
	A[0*9+4] =  *dt;

	A[1*9+3] = -*dt;
	A[1*9+5] =  *dt;
};

void linear_measure(float*X, float* Y, float *H)
{
	float Pn = -65.0;
	float gamma = 2.5;

	// RSSI measurement
	Y[0] = Pn - (10.0 * gamma * log10(sqrt(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0))));

	// x velocity of i wrt i body frame
	Y[1] = X[2];

	// y velocity of i wrt i body frame
	Y[2] = X[3];

	// Orientation of i wrt north
	Y[3] = X[6];

	// x velocity of j wrt j body frame
	Y[4] = cos( X[6] - X[7] ) * X[4] - sin( X[6] - X[7] ) * X[5];

	// y velocity of j wrt  j body frame
	Y[5] = sin( X[6] - X[7] ) * X[4] + cos( X[6] - X[7] ) * X[5];

	// Orientation of j wrt north
	Y[6] = X[7];

	// Height difference
	Y[7] = X[8];

	int n = 9;
	int m = 8;	
	int row, col;

	// Generate the Jacobian Matrix
	for (row = 0 ; row < m ; row++ )
	{
		for (col = 0 ; col < n ; col++ )
		{
			if ((row == 0) && (col == 0 || col == 1 || col == 8 ))
				H[ row*n+col ] = (-gamma*10/log(10))*(X[col]/(pow(X[0],2.0) + pow(X[1],2.0) + pow(X[8],2.0)));
			
			else if (((row == 1) && (col == 2)) ||
				((row == 2) && (col == 3)) ||
				((row == 3) && (col == 6)) ||
				((row == 6) && (col == 7)) ||
				((row == 7) && (col == 8)))
			{ 
				H[ row*n+col ] = 1.0;
			}

			else if ((row == 4) && (col == 4))
				H[ row*n+col ] = cos(X[6]-X[7]);
			else if ((row == 4) && (col == 5))
				H[ row*n+col ] = -sin(X[6]-X[7]);
			else if ((row == 4) && (col == 6))
				H[ row*n+col ] = X[4]*sin(X[7]-X[6]) - X[5] * cos(X[7] - X[6]);
			else if ((row == 4) && (col == 7))
				H[ row*n+col ] = X[4]*sin(X[6]-X[7]) + X[5] * cos(X[6] - X[7]);
			

			else if ((row == 5) && (col == 4))
				H[ row*n+col ] = sin(X[6]-X[7]);
			else if ((row == 5) && (col == 5))
				H[ row*n+col ] = cos(X[6]-X[7]);
			else if ((row == 5) && (col == 6))
				H[ row*n+col ] = X[4]*cos(X[7]-X[6]) + X[5] * sin(X[7] - X[6]);
			else if ((row == 5) && (col == 7))
				H[ row*n+col ] = -X[4]*cos(X[6]-X[7]) + X[5] * sin(X[6] - X[7]);

			else 
				H[ row*n+col ] = 0.0;
		}
	}




};
