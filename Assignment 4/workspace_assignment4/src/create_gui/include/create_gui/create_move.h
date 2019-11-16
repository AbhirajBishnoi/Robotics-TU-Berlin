#include "ros/ros.h"

class CreateMove {

	int req_right_old, req_left_old;

	int axis_right, axis_left;
	int max_right, max_left;

	ros::NodeHandle node;
	//ros::ServiceClient brakeCreate;
	ros::ServiceClient tankCreate;
public:
	CreateMove();

	~CreateMove();
	bool move(int req_right, int req_left);

	bool stop();
};
