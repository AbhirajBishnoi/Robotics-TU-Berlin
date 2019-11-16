/*
 * RosDrawer.h
 *
 *  Created on: 01.10.2010
 *      Author: dermax
 */

#ifndef ROSDRAWER_H_
#define ROSDRAWER_H_

#include "wx/wx.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/LinearMath/Transform.h" //jotelha groovy

//jotelha groovy adaptions:
/*typedef tf::Transform tf::Transform;
typedef tf::Vector3 tf::Vector3;
typedef tf::Quaternion tf::Quaternion;
#define tfScalar tf::Scalar*/

/*#define tf::Transform ft::Transform
#define tf::Vector3 ft::Vector3
#define tf::Quaternion ft::Quaternion
#define tfScalar ft::Scalar*/ 

/**
 * Abstract class for the plug-in to MapScrolledWindow.
 * The idea behind this concept is to make it easy to draw different values receieved over ros on a map.
 * When implementing a new plug-in one would override the constructor and subscribe to some ros topics.
 * You also have to override the OnDraw() function to actually draw something on the map. OnDraw is called every time the
 * window needs to be redrawn (scaled or window size changed) or a message from ros is received.
 * To draw something on the real location you can call toDisplay() with a xy position relative to the map frame.
 */
class RosDrawer
{
protected:
	// pointer to the main ros node
	ros::NodeHandle *m_node;
	// The map display scale
	double m_scale;
	// offset to the top left corner of the map inside the Window
	int m_offset_x, m_offset_y;
	// pointer to the most resent map mesage
	nav_msgs::OccupancyGridConstPtr mapMsg;
public:
	RosDrawer(ros::NodeHandle &node){ m_node=&node;};
	~RosDrawer(){};
	/**
	 * called from MapScrolledWindow when a new map is received
	 */
	void SetMap(const nav_msgs::OccupancyGridConstPtr &mapMsg) {this->mapMsg = mapMsg;};
	/**
	 * called from MapScrolledWindow before OnDraw() is called
	 */
	void SetDisplayParameter(double scale, int offset_x, int offset_y) { m_scale = scale; m_offset_x=offset_x; m_offset_y=offset_y;};
	/**
	 * covert metric position of the map to pixel coordinates in the map
	 */
	wxPoint toDisplay(double metric_x, double metric_y)
	{
		if(!mapMsg) return wxPoint(0,0);
        // get the map transform
		tf::Transform transform(tf::Quaternion(0,0,0,1),tf::Vector3(metric_x,metric_y,0));
		tf::Transform map_transform = GetMapTransform();
		transform = map_transform.inverse()*transform;
		metric_x = transform.getOrigin()[0];
		metric_y = transform.getOrigin()[1];
		return wxPoint(
				m_offset_x - metric_x*m_scale/mapMsg->info.resolution + mapMsg->info.width*m_scale,
				m_offset_y + metric_y*m_scale/mapMsg->info.resolution
				);
	};
	/**
	 * covert pixel coordinates of the map to metric position in the map
	 */
	void toMap(int pixel_x, int pixel_y, double *metric_x, double *metric_y)
	{
		if(!mapMsg) return;
		*metric_x = -(pixel_x - m_offset_x - mapMsg->info.width*m_scale)*mapMsg->info.resolution/m_scale;
		*metric_y = (pixel_y - m_offset_y)*mapMsg->info.resolution/m_scale;
		tf::Transform transform(tf::Quaternion(0,0,0,1),tf::Vector3(*metric_x,*metric_y,0));
		tf::Transform map_transform = GetMapTransform();
		transform = map_transform*transform;
		// now in the right frame
		*metric_x = transform.getOrigin()[0];
		*metric_y = transform.getOrigin()[1];
	};
	/*
	 * returns the transformation between the map origin and the map frame usually 0
	 */
	tf::Transform GetMapTransform()
	{
		tf::Transform map_transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
		if(!mapMsg)	return map_transform;
		map_transform.setOrigin(tf::Vector3(mapMsg->info.origin.position.x, mapMsg->info.origin.position.y, mapMsg->info.origin.position.z));
		map_transform.setRotation(tf::Quaternion(mapMsg->info.origin.orientation.x, mapMsg->info.origin.orientation.y, mapMsg->info.origin.orientation.z, mapMsg->info.origin.orientation.w));
		return map_transform;
	}
	/**
	 * main function to implement
	 */
	virtual void OnDraw(wxDC &dc) = 0;
};



#endif /* ROSDRAWER_H_ */
