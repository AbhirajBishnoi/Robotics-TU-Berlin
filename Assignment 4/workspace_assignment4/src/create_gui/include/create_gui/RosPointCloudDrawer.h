/*
 * RosPointCloudDrawer.h
 *
 *  Created on: 11.10.2010
 *      Author: dermax
 */

#ifndef ROSPOINTCLOUDDRAWER_H_
#define ROSPOINTCLOUDDRAWER_H_


#include "sensor_msgs/PointCloud.h"

class RosPointCloudDrawer : public RosDrawer
{
	ros::Subscriber m_sub_point_cloud;
	sensor_msgs::PointCloudConstPtr m_msg;
public:
	RosPointCloudDrawer(ros::NodeHandle &node, std::string topic_point_cloud = "/likelihood_field"): RosDrawer(node)
	{
		m_sub_point_cloud = node.subscribe(topic_point_cloud, 1, &RosPointCloudDrawer::PointCloudCallback, this);
	};

	void PointCloudCallback(const sensor_msgs::PointCloudConstPtr &msg)
	{
		m_msg = msg;
	};

	void OnDraw(wxDC &dc)
	{
		if(!m_msg) return;

        for(size_t i = 0; i<m_msg->points.size(); i++)
        {
        	dc.SetPen(wxPen(wxColour(m_msg->channels[0].values[i], m_msg->channels[0].values[i], m_msg->channels[0].values[i])));
        	dc.DrawPoint(toDisplay(m_msg->points[i].x, m_msg->points[i].y));
        }

	};
};


#endif /* ROSPOINTCLOUDDRAWER_H_ */
