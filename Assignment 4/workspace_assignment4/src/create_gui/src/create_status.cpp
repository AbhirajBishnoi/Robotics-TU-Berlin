#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include "create_gui/create_status.h"

using namespace std;

	CreateStatus::CreateStatus() {
		sub_status = node.subscribe("sensorPacket",1, &CreateStatus::sensorSubscriber, this);
	}

	CreateStatus::~CreateStatus() {  }

	double CreateStatus::getBatStatus()
	{
		return (double)cur_status.voltage/1000.0;
	}

	void CreateStatus::sensorSubscriber(const rbo_create::SensorPacketConstPtr &msg)
	{
		cur_status = *msg;
	}
