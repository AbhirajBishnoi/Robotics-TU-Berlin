#include "localization/Util.h"

#include <stdlib.h>
#include <math.h>

double Util::gaussian( double x, double std, double mean ) {
	return exp( -(x - mean)*(x - mean) / (2.0*(std * std)));
}

// samples from a gaussian distribution
double Util::gaussianRandom( double mean, double std ) {
	const double norm = 1.0 / (RAND_MAX + 1.0);
	double u = 1.0 - rand() * norm;                  /* can't let u == 0 */
	double v = rand() * norm;
	double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v);
	return mean + std * z;
} 

// samples from a uniform distribution
double Util::uniformRandom(double min, double max) {
	return min + (rand() / (double)RAND_MAX) * (max - min);
}

// returns the shortest difference and direction to go from a to b
double Util::diffAngle(double a, double b) {
	return normalizeTheta(b - a);
}

// returns an angle between -PI and PI
double Util::normalizeTheta(double theta) {

  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}
