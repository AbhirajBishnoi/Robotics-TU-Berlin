#include "ros/ros.h"
#include "rbo_create/SensorPacket.h"

class CreateStatus {

	ros::NodeHandle node;
	ros::Subscriber sub_status;
public:
	rbo_create::SensorPacket cur_status;
	CreateStatus();

	~CreateStatus();
	void sensorSubscriber(const rbo_create::SensorPacketConstPtr &msg);

	double getBatStatus();

};
