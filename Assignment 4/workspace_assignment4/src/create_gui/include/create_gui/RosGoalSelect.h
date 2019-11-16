/*
 * RosGoalSelect.h
 *
 *  Created on: 11.10.2010
 *      Author: dermax
 */

#ifndef ROSGOALSELECT_H_
#define ROSGOALSELECT_H_

#include "RosDrawer.h"
#include "geometry_msgs/PoseStamped.h"
/*
 * publish a PoseStamped topic
 */
class RosGoalSelect : public RosDrawer, wxEvtHandler
{
	bool m_goal_set;
	double m_goal_x,m_goal_y;
	MapScrolledWindow *m_window;
	ros::Publisher m_pub_goal;
	bool m_setting_direction;
	wxPoint m_mouse_point;
	geometry_msgs::PoseStamped m_goal;
public:
	RosGoalSelect(ros::NodeHandle &node, MapScrolledWindow *window, std::string topic_goal="/move_base_simple/goal"): RosDrawer(node), wxEvtHandler()
	{
		m_window = window;
		m_goal_set = false;
		m_setting_direction = false;
		m_pub_goal = node.advertise<geometry_msgs::PoseStamped>(topic_goal,1,false);
	}

	void SetGoal(wxPoint point)
	{
		if(!mapMsg) return;
		int mouse_x=point.x, mouse_y=point.y, map_x,map_y;
		m_window->CalcUnscrolledPosition(mouse_x, mouse_y, &map_x, &map_y);
		double goal_x=0,goal_y=0;
		toMap(map_x, map_y, &goal_x, &goal_y);

		m_goal_x = goal_x;
		m_goal_y = goal_y;

		m_setting_direction = true;
		m_goal_set = false;
		m_goal_set = true;
		m_window->Connect(wxEVT_MOTION,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
		m_window->Connect(wxEVT_LEFT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
		m_window->Connect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);

	}

	void OnMouse(wxMouseEvent &event)
	{
		SetGoal(event.GetPosition());
	}

	void OnMouseMove(wxMouseEvent &event)
	{
		if(m_setting_direction)
		{
			m_window->Refresh();
			m_mouse_point = m_window->CalcUnscrolledPosition(event.GetPosition());
			if(event.GetEventType() == wxEVT_RIGHT_UP)
			{
				m_setting_direction = false;
				m_window->Disconnect(wxEVT_MOTION,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_LEFT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_goal_set = false;
			}
			tf::Quaternion orientation(0,0,0,1);
			int map_x,map_y;
			m_window->CalcUnscrolledPosition(event.GetPosition().x, event.GetPosition().y, &map_x, &map_y);
			double goal_x=0,goal_y=0;
			toMap(map_x, map_y, &goal_x, &goal_y);
			double yaw = atan2(goal_y - m_goal_y, goal_x - m_goal_x);
			orientation.setRPY(0,0,yaw);
			m_goal.header.frame_id = mapMsg->header.frame_id;
			m_goal.pose.position.x = m_goal_x;
			m_goal.pose.position.y = m_goal_y;
			m_goal.pose.position.z = 0;
			m_goal.pose.orientation.w = orientation.getW();
			m_goal.pose.orientation.x = orientation.getX();
			m_goal.pose.orientation.y = orientation.getY();
			m_goal.pose.orientation.z = orientation.getZ();
			if(event.GetEventType() == wxEVT_LEFT_UP)
			{
				m_pub_goal.publish<geometry_msgs::PoseStamped>(m_goal);
				m_window->Disconnect(wxEVT_MOTION,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_LEFT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosGoalSelect::OnMouseMove),NULL,this);
				m_setting_direction = false;
				m_goal_set = true;
			}
		}
	}

	void OnDraw(wxDC &dc)
	{
		if(m_setting_direction)
		{
			dc.DrawLine(toDisplay(m_goal_x,m_goal_y), m_mouse_point);
		}
		if(m_goal_set)
		{
			dc.SetPen(wxPen(*wxRED,2));
			double roll, pitch, yaw;
			tf::Transform(tf::Quaternion(m_goal.pose.orientation.x, m_goal.pose.orientation.y, m_goal.pose.orientation.z, m_goal.pose.orientation.w),tf::Vector3(0,0,0)).getBasis().getRPY(roll, pitch, yaw);
			dc.DrawCircle(toDisplay(m_goal_x, m_goal_y),0.1*m_scale/mapMsg->info.resolution);
			dc.DrawLine(toDisplay(m_goal_x, m_goal_y),toDisplay(m_goal_x+ 0.15*cos(yaw), m_goal_y+0.15*sin(yaw)));
			dc.SetPen(wxNullPen);
		}
	}
};


#endif /* ROSGOALSELECT_H_ */
