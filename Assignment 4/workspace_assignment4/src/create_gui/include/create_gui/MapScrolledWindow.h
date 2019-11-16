/*
 * MapScrolledWindow.h
 *
 *  Created on: 01.10.2010
 *      Author: dermax
 */

#ifndef MAPSCROLLEDWINDOW_H_
#define MAPSCROLLEDWINDOW_H_

#include "RosDrawer.h"
#include "ScrolledImageComponent.h"
#include "nav_msgs/OccupancyGrid.h"

#include <vector>

class MapScrolledWindow : public ScrolledImageComponent
{
	std::vector<RosDrawer*> m_drawer;
	nav_msgs::OccupancyGridConstPtr mapMsg;
public:
	MapScrolledWindow(wxWindow* parent, wxWindowID id, wxImage &image):     ScrolledImageComponent(parent, id, image)
	{

	}

	//virtual ~MapScrolledWindow() = 0;
    void OnDraw(wxDC& dc){
    	DoDraw(dc);
    	int offset_x, offset_y;
    	GetOffset(&offset_x,&offset_y);
    	for(std::vector<RosDrawer*>::iterator it= m_drawer.begin() ; it != m_drawer.end() ; it++)
    	{
    		(*it)->SetDisplayParameter(m_scale, offset_x, offset_y);
    		(*it)->OnDraw(dc);
    	}
    };

    void SetMap(const nav_msgs::OccupancyGridConstPtr &mapMsg)
    {
    	this->mapMsg = mapMsg;
    	wxImage map(mapMsg->info.width, mapMsg->info.height);
    	unsigned char *data = map.GetData();
    	for(size_t i=0;i<mapMsg->data.size();i++)
    	{
			char Occupancy = mapMsg->data[i];
    		if(Occupancy<0)
    		{
        		data[i*3] = data[i*3+1] = data[i*3+2] = 128;
    		} else
    		{
    			if(Occupancy>100) Occupancy =100;
    			data[i*3] = data[i*3+1] = data[i*3+2] = 255 - Occupancy*255/100;
    		}
    	}
    	map = map.Mirror(wxVERTICAL);
    	SetImage(map);
    	for(std::vector<RosDrawer*>::iterator it= m_drawer.begin() ; it != m_drawer.end() ; it++)
    	{
    		(*it)->SetMap(mapMsg);
    	}
    }

    void Add(RosDrawer* drawer)
    {
    	m_drawer.push_back(drawer);
    }

	/**
	 * covert pixel coordinates of the map to metric position in the map
	 */
	void toMap(int pixel_x, int pixel_y, double *metric_x, double *metric_y)
	{
		if(!mapMsg) return;
		int offset_x=0,offset_y=0;
		//GetOffset(&offset_x, &offset_y);
		*metric_x = -(pixel_x - offset_x - mapMsg->info.width*m_scale)*mapMsg->info.resolution/m_scale;
		*metric_y = (pixel_y - offset_y)*mapMsg->info.resolution/m_scale;
		tf::Transform transform(tf::Quaternion(0,0,0,1),tf::Vector3(*metric_x,*metric_y,0));
		tf::Transform map_transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
		map_transform.setOrigin(tf::Vector3(mapMsg->info.origin.position.x, mapMsg->info.origin.position.y, mapMsg->info.origin.position.z));
		map_transform.setRotation(tf::Quaternion(mapMsg->info.origin.orientation.x, mapMsg->info.origin.orientation.y, mapMsg->info.origin.orientation.z, mapMsg->info.origin.orientation.w));
		transform = map_transform*transform;
		// now in the right frame
		*metric_x = transform.getOrigin()[0];
		*metric_y = transform.getOrigin()[1];
	};

};

#endif /* MAPSCROLLEDWINDOW_H_ */
