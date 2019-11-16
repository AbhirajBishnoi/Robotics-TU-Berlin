// A particle filter node for educational purposes

#include <vector>
//#include <btVector3.h> //jotelha groovy

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// roscpp
#include "ros/ros.h"

// Messages that the particle filter needs
#include "localization/PoseWithWeight.h"
#include "localization/PoseWithWeightStamped.h"
#include "localization/PoseWithWeightArray.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/GetMap.h"

#include "std_srvs/Empty.h"
#include "localization/SetInitialPose.h"

// For transform support
//#include "tf/transform_datatypes.h" //jotelha groovy btVector3
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"

#include "localization/ParticleFilter.h"
#include "localization/Util.h"

using namespace localization;

// helper
void poseTFToPoseWithWeightMsg(const tf::Pose& bt, double weight, localization::PoseWithWeight& msg);

class ParticleFilterNode {
public:
	ParticleFilterNode(ros::NodeHandle n);
	~ParticleFilterNode();

private:
	ros::NodeHandle nodeHandle, nodeHandlePrivate;
	tf::TransformListener tfListener;
	tf::TransformBroadcaster transformBroadcaster;
	ros::Time last_time_stamp;
	tf::Transform transform_map_odom;

	// Particle filter
	ParticleFilter* particleFilter;
	boost::mutex pfMutex;

	// subscribe to laser scan
	message_filters::Subscriber<sensor_msgs::LaserScan> laserScanSubscriber;
	tf::MessageFilter<sensor_msgs::LaserScan> laserScanFilter;

	// subscribe to the odometry and remember the last one we integrated
	ros::Subscriber odometrySubscriber;
	tf::Stamped<tf::Pose> lastOdomPose;
	bool bigChangeInPose;
	
	// the corresponding message callbacks
	void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
	void odometryReceived(const nav_msgs::OdometryConstPtr& msg);

	// publish particles, estimated pose and likelihood field for the sensor model
	ros::Publisher posePublisher;
	ros::Publisher particleSetPublisher;
	ros::Publisher importanceParticleSetPublisher;
	ros::Publisher likelihoodFieldPublisher;
	ros::Publisher likelihoodMapPublisher;

	// the corresponding methods for converting the data structures to ROS-messages
	void publishLikelihoodField();
	void publishParticleSet();
	void publishImportanceParticleSet();
	void publishHypothesis();

	// three services are offered: one for publishing all the stuff mentioned above and two for placing the particles in the beginning of a localization run
	ros::ServiceServer publishEverythingService;
	ros::ServiceServer distributeNormalService;
	ros::ServiceServer distributeUniformService;

	// the corresponding service callbacks
	bool publishEverythingRequested(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
	bool distributeNormalRequested(localization::SetInitialPose::Request& req, localization::SetInitialPose::Response& res);
	bool distributeUniformRequested(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

	// we need a map from the map_server
	void requestMap();
	nav_msgs::OccupancyGrid map;
};


ParticleFilterNode::ParticleFilterNode(ros::NodeHandle n) :
	nodeHandle(n),
	nodeHandlePrivate("~"),
	laserScanSubscriber(nodeHandle, "scan", 5),
	laserScanFilter(laserScanSubscriber, tfListener, "odom", 5)
{
	bigChangeInPose = false;

	// Grab params off the param server
	int number_of_particles;
	nodeHandlePrivate.param("number_of_particles", number_of_particles, 500);

	// how should the particle set be initialized?
	bool init_particles_uniform;
	double init_x, init_y, init_theta, init_cov_xx, init_cov_yy, init_cov_thetatheta;
	nodeHandlePrivate.param("init_particles_uniform", init_particles_uniform, true);
	nodeHandlePrivate.param("initial_pose_x", init_x, 0.0);
	nodeHandlePrivate.param("initial_pose_y", init_y, 0.0);
	nodeHandlePrivate.param("initial_pose_theta", init_theta, 0.0);
	nodeHandlePrivate.param("initial_cov_xx", init_cov_xx, .5 * .5);
	nodeHandlePrivate.param("initial_cov_yy", init_cov_yy, .5 * .5);
	nodeHandlePrivate.param("initial_cov_thetatheta", init_cov_thetatheta, (M_PI / 12.0) * (M_PI / 12.0));

	// motion model params
	double alpha1, alpha2, alpha3, alpha4;
	nodeHandlePrivate.param("motionModel_alpha1", alpha1, 0.2);
	nodeHandlePrivate.param("motionModel_alpha2", alpha2, 0.2);
	nodeHandlePrivate.param("motionModel_alpha3", alpha3, 0.2);
	nodeHandlePrivate.param("motionModel_alpha4", alpha4, 0.2);

	// sensor model params -- YOU'll probably need to change these!
	double minLikelihood, std;
	nodeHandlePrivate.param("sensorModel_minLikelihood", minLikelihood, 0.7);
	nodeHandlePrivate.param("sensorModel_std", std, 0.2);


	// get map from ros map_server
	requestMap();

	// create the particle filter
	particleFilter = new ParticleFilter(number_of_particles);

	// configure the motion and sensor model
	particleFilter->setMotionModelOdometry(alpha1, alpha2, alpha3, alpha4);
	particleFilter->setMeasurementModelLikelihoodField(map, minLikelihood, std);

	// initialize filter
	if( init_particles_uniform ) {
		ROS_INFO("Initializing particle filter with a uniform particle distribution across the map.");
		particleFilter->initParticlesUniform();
	}
	else {
		ROS_INFO("Initializing particle filter with a gaussian distribution around (%f, %f, %f) and a standard deviation of (%f, %f, %f).", init_x, init_y, init_theta, init_cov_xx, init_cov_yy, init_cov_thetatheta);
		particleFilter->initParticlesGaussian(init_x, init_y, init_theta, init_cov_xx, init_cov_yy, init_cov_thetatheta);
	}


	// subscribe to  laser and odometry
	laserScanFilter.registerCallback(boost::bind(&ParticleFilterNode::laserReceived, this, _1));
	odometrySubscriber = nodeHandle.subscribe("odom", 5, &ParticleFilterNode::odometryReceived, this);

	// publish the map we're building for the sensor model (as a point cloud - to visualize different reflection probabilities)
	likelihoodFieldPublisher = nodeHandlePrivate.advertise<sensor_msgs::PointCloud > ("likelihood_field", 1);
	likelihoodMapPublisher = nodeHandlePrivate.advertise<nav_msgs::OccupancyGrid > ("likelihood_map", 1);

	posePublisher = nodeHandlePrivate.advertise<geometry_msgs::PoseStamped > ("estimated_pose", 2);
	particleSetPublisher = nodeHandlePrivate.advertise<geometry_msgs::PoseArray > ("particlecloud", 2);
	// we use the PoseWithWeightArray to add the particle weight
	importanceParticleSetPublisher = nodeHandlePrivate.advertise<PoseWithWeightArray > ("importanceparticlecloud", 2);


	// advertise all the services the particle filter is offering
	publishEverythingService = nodeHandlePrivate.advertiseService("publish_everything", &ParticleFilterNode::publishEverythingRequested, this);
	distributeNormalService = nodeHandlePrivate.advertiseService("distribute_normal", &ParticleFilterNode::distributeNormalRequested, this);
	distributeUniformService = nodeHandlePrivate.advertiseService("distribute_uniform", &ParticleFilterNode::distributeUniformRequested, this);

	bigChangeInPose = true;
	publishLikelihoodField();
	publishParticleSet();
	publishHypothesis();
}

ParticleFilterNode::~ParticleFilterNode() {
	// delete everything allocated in constructor
	delete particleFilter;
}

void ParticleFilterNode::requestMap() {
	// get map via map_server
	nav_msgs::GetMap::Request req;
	nav_msgs::GetMap::Response resp;

	ROS_INFO("Requesting the map...");

	while (!ros::service::call("static_map", req, resp) && nodeHandle.ok()) {
		ROS_WARN("Request for map failed; trying again...");
		ros::Duration d(2.0);
		d.sleep();
	}

	ROS_INFO("Received a %d X %d map @ %.3f m/pix\n", resp.map.info.width, resp.map.info.height, resp.map.info.resolution);

	map = resp.map;
}

void ParticleFilterNode::publishLikelihoodField() {
	// publish the likelihood field as a point cloud -- every grid cell is represented by a point, likelihoods by their color
	sensor_msgs::PointCloud likelihoodField;

	likelihoodField.header.frame_id = "map";
	likelihoodField.header.stamp = this->last_time_stamp;

	int w, h;
	double res;
	double* data = this->particleFilter->getLikelihoodField(w, h, res);

	likelihoodField.points.resize(w * h); //jotelha groovy
	likelihoodField.channels.resize(1);
	likelihoodField.channels[0].name = "intensities";
	likelihoodField.channels[0].values.resize(w * h);

	for (int x = 0; x < w; x++) {
		for (int y = 0; y < h; y++) {
			likelihoodField.points[x + y * w].x = res * x;
			likelihoodField.points[x + y * w].y = res * y;
			likelihoodField.points[x + y * w].z = 0.0;
			likelihoodField.channels[0].values[x + y * w] = data[x + y * w];
		}
	}

	this->likelihoodFieldPublisher.publish(likelihoodField);

	// publish the likelihood filed as an occupancy grid map -- create_gui will visualize this correctly, rviz breaks it down to a 3-colored image (free,occupied,unknown)
	nav_msgs::OccupancyGrid likelihoodMap;
	likelihoodMap.header.frame_id = "map";
	likelihoodMap.header.stamp = last_time_stamp;
	likelihoodMap.info.origin = map.info.origin;
//	ROS_INFO("origin=(%f,%f,%f)\n",likelihoodMap.info.origin.position.x,likelihoodMap.info.origin.position.y,likelihoodMap.info.origin.position.z);
	likelihoodMap.info.height = h;
	likelihoodMap.info.width = w;
	likelihoodMap.info.resolution = res;
	
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			likelihoodMap.data.push_back((int8_t)(exp(data[x + y * w])*100));
		}
	}
	ROS_INFO("publish likelihood map");
	this->likelihoodMapPublisher.publish(likelihoodMap);
}

void ParticleFilterNode::publishParticleSet() {
	pfMutex.lock();

	// publish the particles as a set of arrows
	geometry_msgs::PoseArray particleCloudMsg;
	particleCloudMsg.header.stamp = this->last_time_stamp;
	particleCloudMsg.header.frame_id = "map";
	particleCloudMsg.poses.resize(this->particleFilter->getNumberOfParticles()); //jotelha groovy

	int i = 0;
	for (std::vector<Particle*>::iterator it = this->particleFilter->getParticleSet()->begin(); it != this->particleFilter->getParticleSet()->end(); it++) {
		tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw((*it)->theta), tf::Vector3((*it)->x, (*it)->y, 0)), particleCloudMsg.poses[i++]);
	}

	pfMutex.unlock();

	// subtracting base to odom from map to base and send map to odom instead 
	// this way we send a correction to the odom pose
	tf::StampedTransform transform_base_link_odom;
	try{
		//ros::Time t = ros::Time::now();
		ros::Time t = ros::Time(0);
		tfListener.waitForTransform("base_link", "odom",
				t, ros::Duration(10.0),ros::Duration(0.1));

      		tfListener.lookupTransform("base_link", "odom",
    		  		t, transform_base_link_odom);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

	tf::Transform transform_map_base;
	transform_map_base.setOrigin( tf::Vector3(this->particleFilter->getBestHypothesis()->x, this->particleFilter->getBestHypothesis()->y, 0.0) );
  transform_map_base.setRotation( tf::createQuaternionFromYaw(this->particleFilter->getBestHypothesis()->theta) ); 
	//transform_map_base.setRotation( tf::Quaternion(this->particleFilter->getBestHypothesis()->theta, 0, 0) );

	this->transform_map_odom = transform_map_base*transform_base_link_odom;

	this->particleSetPublisher.publish(particleCloudMsg);
}



void poseTFToPoseWithWeightMsg(const tf::Pose& bt, double weight, localization::PoseWithWeight& msg) {
	tf::pointTFToMsg(bt.getOrigin(), msg.pose.position);
	tf::quaternionTFToMsg(bt.getRotation(), msg.pose.orientation);
	msg.weight = weight;
}

void ParticleFilterNode::publishImportanceParticleSet() {
	pfMutex.lock();

	// publish the particles as a set of arrows
	PoseWithWeightArray particleCloudMsg;
	particleCloudMsg.header.stamp = last_time_stamp;
	particleCloudMsg.header.frame_id = "map";
	particleCloudMsg.poses.resize(this->particleFilter->getNumberOfParticles()); //jotelha groovy

	int i = 0;
	for (std::vector<Particle*>::iterator it = this->particleFilter->getParticleSet()->begin(); it != this->particleFilter->getParticleSet()->end(); it++) {
		tf::Pose pose(tf::createQuaternionFromYaw((*it)->theta), tf::Vector3((*it)->x, (*it)->y, 0)); //jotelha groovy
		poseTFToPoseWithWeightMsg(pose, (*it)->weight, particleCloudMsg.poses[i++]);
		//tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw((*it)->theta), tf::Vector3((*it)->x, (*it)->y, 0)),
				//);
	}

	pfMutex.unlock();

	// and publish the transform between map and odometry
	tf::StampedTransform transform_base_link_odom;
	try{
		//ros::Time t = ros::Time::now();
		ros::Time t = ros::Time(0);
		tfListener.waitForTransform("base_link", "odom",
				t, ros::Duration(10.0),ros::Duration(0.1));

      tfListener.lookupTransform("base_link", "odom",
    		  t, transform_base_link_odom);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

//	tf::Transform transform_map_base;
//	transform_map_base.setOrigin( tf::Vector3(this->particleFilter->getBestHypothesis()->x, this->particleFilter->getBestHypothesis()->y, 0.0) );
//	transform_map_base.setRotation( tf::Quaternion(this->particleFilter->getBestHypothesis()->theta, 0, 0) );
//
//	this->transform_map_odom =  transform_map_base*transform_base_link_odom;

	this->importanceParticleSetPublisher.publish(particleCloudMsg);
}

void ParticleFilterNode::publishHypothesis() {
	// publish the estimated robot pose as an arrow
	geometry_msgs::PoseStamped estimatedPoseMsg;
	estimatedPoseMsg.header.frame_id = "map";
	estimatedPoseMsg.header.stamp = this->last_time_stamp;
	estimatedPoseMsg.pose.position.x = this->particleFilter->getBestHypothesis()->x;
	estimatedPoseMsg.pose.position.y = this->particleFilter->getBestHypothesis()->y;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(this->particleFilter->getBestHypothesis()->theta), estimatedPoseMsg.pose.orientation);
	this->posePublisher.publish(estimatedPoseMsg);

	this->transformBroadcaster.sendTransform(tf::StampedTransform(transform_map_odom, last_time_stamp, "map","odom"));

}

bool ParticleFilterNode::publishEverythingRequested(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	publishLikelihoodField();
	publishParticleSet();
	publishImportanceParticleSet();
	publishHypothesis();

	return true;
}

bool ParticleFilterNode::distributeNormalRequested(localization::SetInitialPose::Request& req, localization::SetInitialPose::Response& res) {
	pfMutex.lock();
	particleFilter->initParticlesGaussian( req.x, req.y, req.theta, req.var_x, req.var_y, req.var_theta );
	pfMutex.unlock();

	publishParticleSet();

	return true;
}

bool ParticleFilterNode::distributeUniformRequested(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
	pfMutex.lock();
	particleFilter->initParticlesUniform();
	pfMutex.unlock();

	publishParticleSet();

	return true;
}

void ParticleFilterNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laserScan) {
//	ROS_INFO("LaserScan received");
	this->last_time_stamp = laserScan->header.stamp;

	// if the robot has moved, update the filter
	if (this->bigChangeInPose) {
		pfMutex.lock();

		// correction step
		this->particleFilter->measurementModel(laserScan);
		// resample the particles
		this->particleFilter->resample();
		pfMutex.unlock();
		this->bigChangeInPose = false;
		// Publish the resulting cloud and the corresponding hypothesis
		this->publishParticleSet();
	}

	this->publishHypothesis();

}

void ParticleFilterNode::odometryReceived(const nav_msgs::OdometryConstPtr& msg) {
//	ROS_INFO("Odometry reading received");
	this->last_time_stamp = msg->header.stamp;

	// calculate the difference to the last integrated pose
	double oldX = lastOdomPose.getOrigin().x();
	double oldY = lastOdomPose.getOrigin().y();
	double oldTheta = tf::getYaw(lastOdomPose.getRotation());

	double newX = msg->pose.pose.position.x;
	double newY = msg->pose.pose.position.y;
	double newTheta = tf::getYaw(msg->pose.pose.orientation);

	// how much did the robot move since the last filter step?
	double deltaX = newX - oldX;
	double deltaY = newY - oldY;
	double deltaTheta = Util::diffAngle(oldTheta, newTheta);


	this->bigChangeInPose = fabs(deltaX) > 0.025 || fabs(deltaY) > 0.025 || fabs(deltaTheta) > 0.1;

	// see if we should update the filter
	if (this->bigChangeInPose) {
		pfMutex.lock();
		this->particleFilter->sampleMotionModel(oldX, oldY, oldTheta, newX, newY, newTheta);
		pfMutex.unlock();
		// save this odometry pose
		tf::poseMsgToTF(msg->pose.pose, this->lastOdomPose);

		this->publishParticleSet();
	}
	this->publishHypothesis();
	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "particle_filter");

	ros::NodeHandle nh;

	ParticleFilterNode pfn(nh);

	ros::spin();

	return 0;
}
