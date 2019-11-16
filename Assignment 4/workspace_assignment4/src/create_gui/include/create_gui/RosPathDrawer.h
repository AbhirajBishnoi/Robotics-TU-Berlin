/*
 * RosPathDrawer.h
 *
 *  Created on: 11.10.2010
 *      Author: dermax
 */

#ifndef ROSPATHDRAWER_H_
#define ROSPATHDRAWER_H_

#include "RosDrawer.h"
#include "nav_msgs/Path.h"
/**
 * subscribes to a path and draws it as a line on the map
 */
class RosPathDrawer: public RosDrawer
{
	ros::Subscriber m_sub_path;
	wxPen m_pen;
	nav_msgs::PathConstPtr m_path_msg;
	tf::TransformListener *m_listener;

public:
	RosPathDrawer(ros::NodeHandle &node, std::string topic_path="/path", wxPen pen=wxNullPen): RosDrawer(node)
	{
		m_sub_path = node.subscribe(topic_path,1,&RosPathDrawer::PathCallback,this);
		m_pen = pen;
		m_listener = new tf::TransformListener(node);

	};

	~RosPathDrawer()
	{
		delete m_listener;
	};
	void PathCallback(const nav_msgs::PathConstPtr &msg)
	{
		m_path_msg = msg;
	};

	void OnDraw(wxDC &dc)
	{
    	// check if we have a vaild map
    	if(!mapMsg) return;
    	if(!m_path_msg) return;
    	// look up the transform from the map to the robot
        tf::StampedTransform transform;
        try{
          m_listener->lookupTransform(mapMsg->header.frame_id, m_path_msg->header.frame_id,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return;
        }

		dc.SetPen(m_pen);
		tf::Transform path_point;
		tf::Transform path_point_last;
		for(size_t i=0;i<m_path_msg->poses.size();i++)
		{
			path_point.setRotation(tf::Quaternion(m_path_msg->poses[i].pose.orientation.x,
					m_path_msg->poses[i].pose.orientation.y,
					m_path_msg->poses[i].pose.orientation.z,
					m_path_msg->poses[i].pose.orientation.w));
			path_point.setOrigin(tf::Vector3(m_path_msg->poses[i].pose.position.x,
					m_path_msg->poses[i].pose.position.y,
					m_path_msg->poses[i].pose.position.z));
			path_point = transform*path_point;
			if(i>0)
			{
				dc.DrawLine(toDisplay(path_point_last.getOrigin()[0],path_point_last.getOrigin()[1]),toDisplay(path_point.getOrigin()[0],path_point.getOrigin()[1]));
			}
			path_point_last = path_point;
		}
	};

};

#endif /* ROSPATHDRAWER_H_ */
