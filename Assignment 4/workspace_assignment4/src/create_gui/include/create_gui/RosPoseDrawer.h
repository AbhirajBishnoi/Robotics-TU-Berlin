/*
 * RosPoseDrawer.h
 *
 *  Created on: 11.10.2010
 *      Author: dermax
 */

#ifndef ROSPOSEDRAWER_H_
#define ROSPOSEDRAWER_H_

#include "RosDrawer.h"
#include "geometry_msgs/PoseStamped.h"

class RosPoseDrawer : public RosDrawer
{
	ros::Subscriber m_sub_pose;
	geometry_msgs::PoseStampedConstPtr m_msg;
	double m_arrow_length;
	double m_arrow_angle;
public:
	RosPoseDrawer(ros::NodeHandle &node, std::string topic_pose = "/estimated_pose"): RosDrawer(node)
	{
		m_sub_pose = node.subscribe(topic_pose, 1, &RosPoseDrawer::PoseCallback, this);
		m_arrow_length = 0.6;
		m_arrow_angle = 0.3;
	};

	void PoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
	{
		m_msg = msg;
	};

	void OnDraw(wxDC &dc)
	{
		if(!m_msg) return;
		if(!mapMsg) return;
		dc.SetPen(wxPen(*wxGREEN,3));
		tf::Transform point;
		tfScalar roll, pitch, yaw;

		point.setRotation(tf::Quaternion(m_msg->pose.orientation.x,
					m_msg->pose.orientation.y,
					m_msg->pose.orientation.z,
					m_msg->pose.orientation.w));
		point.setOrigin(tf::Vector3(m_msg->pose.position.x,
					m_msg->pose.position.y,
					m_msg->pose.position.z));
		point.getBasis().getEulerYPR(yaw, pitch, roll);
		dc.DrawLine(toDisplay(m_msg->pose.position.x, m_msg->pose.position.y), toDisplay(m_msg->pose.position.x + m_arrow_length*cos(yaw), m_msg->pose.position.y + m_arrow_length*sin(yaw)));
		dc.DrawLine(toDisplay(m_msg->pose.position.x + m_arrow_length*cos(yaw), m_msg->pose.position.y + m_arrow_length*sin(yaw)), toDisplay(m_msg->pose.position.x + m_arrow_length*cos(yaw+m_arrow_angle)*3/4, m_msg->pose.position.y + m_arrow_length*sin(yaw+m_arrow_angle)*3/4));
		dc.DrawLine(toDisplay(m_msg->pose.position.x + m_arrow_length*cos(yaw), m_msg->pose.position.y + m_arrow_length*sin(yaw)), toDisplay(m_msg->pose.position.x + m_arrow_length*cos(yaw-m_arrow_angle)*3/4, m_msg->pose.position.y + m_arrow_length*sin(yaw-m_arrow_angle)*3/4));
	};
};

#endif /* ROSPOSEDRAWER_H_ */
