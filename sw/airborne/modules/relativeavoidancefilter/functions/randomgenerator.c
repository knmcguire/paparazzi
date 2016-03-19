#include "randomgenerator.h"

/* Initilizer function.

Run this at the beginning of the code and before running any of the functions  below.
It will create an initial random seed that is used later on, tuned to the current time AND to the process ID
(this is because it was found that only using the time is not sufficient, and two nodes spawned at the same time will actually end with the same seed unless the process ID is also included).
*/
void randomgen_init()
{
	// srand( (unsigned) time(NULL) * getpid() );
	float *temp;
	srand( temp );

	// time_t now;
 //  	now = time(NULL);
 //  	struct tm *ts;
 //  	ts = localtime(&now);
 //  	srand(ts->tm_sec);
}

/* Get a random value of type float between a min and max */
float getrand_float(float min, float max)
{
	return min + ((float)rand() / ( RAND_MAX / (max - min) ) ) ;
};

/* Get a random value of type int between a min and a max */
int getrand_int(int min, int max)
{
	return min + (rand() / ( RAND_MAX / (max - min) ) ) ;
};
