/*
 * RosRobotDrawer.h
 *
 *  Created on: 05.10.2010
 *      Author: dermax
 */

#ifndef ROSROBOTDRAWER_H_
#define ROSROBOTDRAWER_H_

#include "RosDrawer.h"
#include <tf/transform_listener.h>

/**
 * @brief draws a circular robot on the map
 */
class RosRobotDrawer: public RosDrawer
{
	std::string m_robot_frame;
	double m_robot_diameter;
	tf::TransformListener *m_listener;
public:
	RosRobotDrawer(ros::NodeHandle &node, std::string robot_frame = "/base_link", double diameter=0.15): RosDrawer(node)
	{
		m_robot_frame = robot_frame;
		m_robot_diameter = diameter;
		m_listener = new tf::TransformListener(node);
	};
	~RosRobotDrawer() {delete m_listener;};

	void OnDraw(wxDC &dc)
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

		tfScalar roll, pitch, yaw;
		transform.getBasis().getEulerYPR(yaw, pitch, roll);

		double robot_x = transform.getOrigin()[0];
		double robot_y = transform.getOrigin()[1];
		// start drawing
		wxPen pen(*wxGREEN, 3); // red pen of width 5
		dc.SetPen(pen);
		dc.DrawCircle(toDisplay(robot_x, robot_y), m_robot_diameter*m_scale/mapMsg->info.resolution);
		dc.DrawLine(toDisplay(robot_x, robot_y), toDisplay(robot_x + m_robot_diameter*cos(yaw), robot_y + m_robot_diameter*sin(yaw)));
		dc.SetPen(wxNullPen);
    };
};


#endif /* ROSROBOTDRAWER_H_ */
