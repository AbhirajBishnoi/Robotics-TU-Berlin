#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include <cstdlib>
#include <ctime>

class Particle {

public:
	Particle( double x = 0, double y = 0, double theta = 0, double weight = 0 ) {
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->weight = weight;
	};
	
	Particle(Particle* p) {
		this->x = p->x;
		this->y = p->y;
		this->theta = p->theta;
		this->weight = p->weight;
	};

	void setZero() {
		x = y = theta = weight = 0.0;
	}

	// pose hypothesis
	double x, y, theta;

	// particle weight used for resampling
	double weight;
};

class ParticleFilter {

private:
	// the particle set, a particle is (x, y, theta, weight), in world coordinates
	std::vector<Particle*> particleSet;
	int numberOfParticles;
	double sumOfParticleWeights;

	Particle* bestHypothesis;

	// the motion model (odometry-based)
	void sampleMotionModelOdometry( double oldX, double oldY, double oldTheta, double newX, double newY, double newTheta );

	// motion model error parameters
	double odomAlpha1, odomAlpha2, odomAlpha3, odomAlpha4;

	// the sensor model (scan-based)
	double* likelihoodField;
	int likelihoodFieldWidth, likelihoodFieldHeight;
	double likelihoodFieldResolution;
	int laserSkip;
	
	void likelihoodFieldRangeFinderModel( const sensor_msgs::LaserScanConstPtr& laserScan );

	double* distMap;

	// Calculate the binary distance map (also known as distance transform).
	// The distance map contains in every cell the distance from this cell
	// to the next obstacle.
	// After calculation, the distance map will be stored in the member variable distMap.
	void calculateDistanceMap(const nav_msgs::OccupancyGrid& map);

	// Compute the index to address a one-dimensional array from a
	// two-dimensional coordinate (x,y)
	int computeMapIndex(int width, int height, int x, int y);


public:
	ParticleFilter( int numberOfParticles );
	~ParticleFilter();
	
	int getNumberOfParticles();
	std::vector<Particle*>* getParticleSet();
	
	void initParticlesUniform();
	void initParticlesGaussian( double mean_x, double mean_y, double mean_theta, double std_xx, double std_yy, double std_tt );
	
	// integrate odometry u (motion model)
	void sampleMotionModel( double oldX, double oldY, double oldTheta, double newX, double newY, double newTheta );
	void setMotionModelOdometry( double alpha1, double alpha2, double alpha3, double alpha4 );

	// integrate laser measurement (sensor model)
	void measurementModel( const sensor_msgs::LaserScanConstPtr& laserScan );

	// compute the initial likehood field given the map occupancy grid
	// @param double zRand 		the minimum likelihood of every cell (pRand=1, zHit+zRand=1)
	// @param double sigmaHit 	the standard deviation of the gaussian likelihood distribution (mean is 0)
	void setMeasurementModelLikelihoodField( const nav_msgs::OccupancyGrid& map, double zRand, double sigmaHit );
	double* getLikelihoodField( int& width, int& height, double& resolution );
	
	// resample (importance sampling)
	void resample();
	
	// pose estimate
	Particle* getBestHypothesis();

};

#endif
