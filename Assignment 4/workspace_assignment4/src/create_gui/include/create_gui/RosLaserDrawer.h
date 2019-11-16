/*
 * RosLaserDrawer.h
 *
 *  Created on: 05.10.2010
 *      Author: dermax
 */

#ifndef ROSLASERDRAWER_H_
#define ROSLASERDRAWER_H_

#include "RosDrawer.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
/**
 * @brief draws a laser scan on the map
 */
class RosLaserDrawer: public RosDrawer
{
	std::string m_topic_laser;
	ros::Subscriber m_sub_laser;
	sensor_msgs::LaserScanConstPtr m_laser_data;
	tf::TransformListener *m_listener;
public:
	RosLaserDrawer(ros::NodeHandle &node, std::string topic_laser="/scan"): RosDrawer(node)
	{
		m_topic_laser = topic_laser;
		m_sub_laser = m_node->subscribe(m_topic_laser,1,&RosLaserDrawer::LaserCallback, this);
		m_listener = new tf::TransformListener(node);
	};
	~RosLaserDrawer() {delete m_listener;};

	void LaserCallback(const sensor_msgs::LaserScanConstPtr &laserMsg )
    {
    	m_laser_data=laserMsg;
    };

    void OnDraw(wxDC &dc)
    {
    	// check if data is available
    	if(!mapMsg) return;
    	if(!m_laser_data) return;
    	// get the transform from the map to the laser scanner
        tf::StampedTransform transform;
        try{
          m_listener->lookupTransform(mapMsg->header.frame_id, m_laser_data->header.frame_id,
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          return;
        }
        // get the robot orientation
		tfScalar roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw);

		// start drawing
		wxPen pen(*wxBLUE, 2); // red pen of width 2
	    dc.SetPen(pen);
		for(size_t index=0;index<m_laser_data->ranges.size();index++)
		{
				double angle = m_laser_data->angle_min + index*m_laser_data->angle_increment;
				double laser_point_x = m_laser_data->ranges[index]*cos(angle+yaw)+transform.getOrigin()[0];
				double laser_point_y = m_laser_data->ranges[index]*sin(angle+yaw)+transform.getOrigin()[1];
				dc.DrawCircle(toDisplay(laser_point_x, laser_point_y),1);
		}
		dc.SetPen(wxNullPen);
    };
};


#endif /* ROSLASERDRAWER_H_ */
