/*
 * RosRobotPathDrawer.h
 *
 *  Created on: 18.10.2010
 *      Author: dermax
 */

#ifndef ROSROBOTPATHDRAWER_H_
#define ROSROBOTPATHDRAWER_H_

#include "RosDrawer.h"
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
/**
 * @brief draws a circular robot on the map
 */
class RosRobotPathDrawer: public RosDrawer
{
	std::string m_robot_frame;
	tf::TransformListener *m_listener;
	std::vector<tf::Transform> m_robot_path;
	ros::Subscriber m_sub_odm;
public:
	RosRobotPathDrawer(ros::NodeHandle &node, std::string robot_frame = "/base_link", double diameter=0.15): RosDrawer(node)
	{
		m_robot_frame = robot_frame;
		m_listener = new tf::TransformListener(node);
		m_robot_path.clear();
		m_sub_odm = node.subscribe("/odom",1,&RosRobotPathDrawer::OdometryCallback,this);
	};
	~RosRobotPathDrawer() {delete m_listener;};

	void OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
	{
    	// check if we have a vaild map
    	if(!mapMsg) return;
    	// look up the transform from the map to the robot
        tf::StampedTransform transform;
        try{
          m_listener->lookupTransform(mapMsg->header.frame_id, m_robot_frame,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return;
        }
        m_robot_path.push_back(transform);
	}

	void ClearPath()
	{
		m_robot_path.clear();
	}

	void OnDraw(wxDC &dc)
    {
    	// check if we have a vaild map
    	if(!mapMsg) return;

		// start drawing
		wxPen pen(*wxGREEN, 3);
		dc.SetPen(pen);
		for(size_t i = 1; i<m_robot_path.size(); i++)
		{
			dc.DrawLine(toDisplay(m_robot_path[i-1].getOrigin()[0], m_robot_path[i-1].getOrigin()[1]),toDisplay(m_robot_path[i].getOrigin()[0], m_robot_path[i].getOrigin()[1]));

		}
		dc.SetPen(wxNullPen);
    };
};

#endif /* ROSROBOTPATHDRAWER_H_ */
