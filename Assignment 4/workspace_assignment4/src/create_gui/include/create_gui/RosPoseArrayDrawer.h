/*
 * RosPoseArrayDrawer.h
 *
 *  Created on: 11.10.2010
 *      Author: dermax
 */

#ifndef ROSPOSEARRAYDRAWER_H_
#define ROSPOSEARRAYDRAWER_H_


#include "geometry_msgs/PoseArray.h"

class RosPoseArrayDrawer : public RosDrawer
{
	ros::Subscriber m_sub_pose_array;
	geometry_msgs::PoseArrayConstPtr m_msg;
	double m_arrow_length;
	double m_arrow_angle;
public:
	RosPoseArrayDrawer(ros::NodeHandle &node, std::string topic_pose_array = "/particlecloud"): RosDrawer(node)
	{
		m_sub_pose_array = node.subscribe(topic_pose_array, 1, &RosPoseArrayDrawer::PoseArrayCallback, this);
		m_arrow_length = 0.2;
		m_arrow_angle = 0.3;
	};

	void PoseArrayCallback(const geometry_msgs::PoseArrayConstPtr &msg)
	{
		m_msg = msg;
	};

	void OnDraw(wxDC &dc)
	{
		if(!m_msg) return;
		if(!mapMsg) return;
		dc.SetPen(wxPen(*wxRED,1));
		tf::Transform point;
		tfScalar roll, pitch, yaw;

		for(size_t i=0; i<m_msg->poses.size(); i++)
		{
			point.setRotation(tf::Quaternion(m_msg->poses[i].orientation.x,
					m_msg->poses[i].orientation.y,
					m_msg->poses[i].orientation.z,
					m_msg->poses[i].orientation.w));
			point.setOrigin(tf::Vector3(m_msg->poses[i].position.x,
					m_msg->poses[i].position.y,
					m_msg->poses[i].position.z));
			point.getBasis().getEulerYPR(yaw, pitch, roll);
			dc.DrawLine(toDisplay(m_msg->poses[i].position.x, m_msg->poses[i].position.y), toDisplay(m_msg->poses[i].position.x + m_arrow_length*cos(yaw), m_msg->poses[i].position.y + m_arrow_length*sin(yaw)));
			dc.DrawLine(toDisplay(m_msg->poses[i].position.x + m_arrow_length*cos(yaw), m_msg->poses[i].position.y + m_arrow_length*sin(yaw)), toDisplay(m_msg->poses[i].position.x + m_arrow_length*cos(yaw+m_arrow_angle)*3/4, m_msg->poses[i].position.y + m_arrow_length*sin(yaw+m_arrow_angle)*3/4));
			dc.DrawLine(toDisplay(m_msg->poses[i].position.x + m_arrow_length*cos(yaw), m_msg->poses[i].position.y + m_arrow_length*sin(yaw)), toDisplay(m_msg->poses[i].position.x + m_arrow_length*cos(yaw-m_arrow_angle)*3/4, m_msg->poses[i].position.y + m_arrow_length*sin(yaw-m_arrow_angle)*3/4));
		}
	};
};


#endif /* ROSPOSEARRAYDRAWER_H_ */
