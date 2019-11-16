#include "wx/wx.h"
#include <iostream>
#include <string>

#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

#include "create_gui/MapScrolledWindow.h"
#include "create_gui/RosRobotDrawer.h"
#include "create_gui/RosLaserDrawer.h"
#include "create_gui/RosPathDrawer.h"
#include "create_gui/RosGoalSelect.h"

class RosMousePlanner : public RosDrawer, wxEvtHandler
{
	bool m_start_set;
	double m_goal_x,m_goal_y;
	MapScrolledWindow *m_window;
	ros::Publisher m_pub_plan;
	bool m_setting_direction;
	wxPoint m_mouse_point;
	nav_msgs::Path m_path;
	size_t m_path_index;
public:
	RosMousePlanner(ros::NodeHandle &node, MapScrolledWindow *window, std::string topic_plan="/mouse_plan"): RosDrawer(node), wxEvtHandler()
	{
		m_window = window;
		m_start_set = false;
		m_setting_direction = false;
		m_pub_plan = node.advertise<nav_msgs::Path>(topic_plan,1,false);
	}

	void SetStart(wxPoint point)
	{
		if(!mapMsg) return;
		m_path_index = 0;

		int mouse_x=point.x, mouse_y=point.y, map_x,map_y;
		m_window->CalcUnscrolledPosition(mouse_x, mouse_y, &map_x, &map_y);
		double goal_x=0,goal_y=0;
		toMap(map_x, map_y, &goal_x, &goal_y);

		m_goal_x = goal_x;
		m_goal_y = goal_y;

		m_path.poses.clear();
		m_path.header.frame_id = mapMsg->header.frame_id;

		m_setting_direction = true;
		m_start_set = false;

		m_window->Connect(wxEVT_MOTION,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Connect(wxEVT_LEFT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Connect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Connect(wxEVT_LEFT_DCLICK,wxEventHandler(RosMousePlanner::OnPathFinished),NULL,this);
	}

	void OnMouse(wxMouseEvent &event)
	{
		SetStart(event.GetPosition());
	}

	void OnCancel(wxEvent &event)
	{

	}

	void OnPathFinished(wxEvent &event)
	{
		m_pub_plan.publish<nav_msgs::Path>(m_path);
		m_window->Disconnect(wxEVT_MOTION,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Disconnect(wxEVT_LEFT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Disconnect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
		m_window->Disconnect(wxEVT_RIGHT_DCLICK,wxEventHandler(RosMousePlanner::OnPathFinished),NULL,this);
		m_setting_direction = false;
	}

	void OnMouseMove(wxMouseEvent &event)
	{
			m_window->Refresh();
			m_mouse_point = m_window->CalcUnscrolledPosition(event.GetPosition());
			if(event.GetEventType() == wxEVT_RIGHT_UP)
			{
				m_setting_direction = false;
				m_start_set = false;
				m_path.poses.clear();
				m_window->Disconnect(wxEVT_MOTION,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_LEFT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);
				m_window->Disconnect(wxEVT_RIGHT_UP,wxMouseEventHandler(RosMousePlanner::OnMouseMove),NULL,this);

			}
			if((event.GetEventType() == wxEVT_LEFT_UP))
			{
				tf::Quaternion orientation(0,0,0,1);
				int map_x,map_y;
				m_window->CalcUnscrolledPosition(event.GetPosition().x, event.GetPosition().y, &map_x, &map_y);
				double goal_x=0,goal_y=0;
				toMap(map_x, map_y, &goal_x, &goal_y);
				double yaw = atan2(goal_y - m_goal_y, goal_x - m_goal_x);
				orientation.setRPY(0,0,yaw);

				geometry_msgs::PoseStamped pose;
				pose.header.frame_id = mapMsg->header.frame_id;

				pose.pose.position.x = m_goal_x;
				pose.pose.position.y = m_goal_y;
				pose.pose.position.z = 0;
				pose.pose.orientation.w = orientation.getW();
				pose.pose.orientation.x = orientation.getX();
				pose.pose.orientation.y = orientation.getY();
				pose.pose.orientation.z = orientation.getZ();

				m_path.poses.push_back(pose);
				m_goal_x = goal_x;
				m_goal_y = goal_y;
				m_start_set = true;
			}

	}

	void OnDraw(wxDC &dc)
	{

		if(m_setting_direction)
		{
			dc.DrawLine(toDisplay(m_goal_x,m_goal_y), m_mouse_point);
		}
		if(m_start_set)
		{
			dc.SetPen(wxPen(*wxRED,2));
			size_t i;
			for(i=1; i<m_path.poses.size(); i++)
			{
				dc.DrawCircle(toDisplay(m_path.poses[i-1].pose.position.x,m_path.poses[i-1].pose.position.y), 3);
				dc.DrawLine(toDisplay(m_path.poses[i-1].pose.position.x,m_path.poses[i-1].pose.position.y), toDisplay(m_path.poses[i].pose.position.x,m_path.poses[i].pose.position.y));
			}
			if(m_path.poses.size()>0)
			{
				dc.DrawCircle(toDisplay(m_goal_x, m_goal_y), 3);
				dc.DrawLine(toDisplay(m_path.poses[i-1].pose.position.x,m_path.poses[i-1].pose.position.y), toDisplay(m_goal_x, m_goal_y));
			}

		}
	}
};


class MapPlannerApp: public wxApp
{
    wxFrame *frame;
    MapScrolledWindow* my_image;
    ros::Subscriber map_subscriber;
    ros::NodeHandle node;
	wxTimer	*timer_ros;
	wxMenu *m_mouse_menu;
	RosGoalSelect *m_drawer_goal;
	RosMousePlanner *m_mouse_planner;

	wxPoint m_right_mouse_click_pos;
	wxMenuBar *m_menubar;
    wxMenu *m_menu_view;

public:

    bool OnInit()
    {
    	// subscribe to the map topic
    	map_subscriber = node.subscribe("/map", 1, &MapPlannerApp::MapCallback, this);

        wxInitAllImageHandlers();
        wxSize inital_size(512,512);
        // sizer with the map on bottom and buttons on top
        wxFlexGridSizer *sizer_grid = new  wxFlexGridSizer(2,1,0,0);
        // sizer for the frame
        wxBoxSizer *sizer_box = new wxBoxSizer(wxHORIZONTAL);
        // top part with zoom buttons and stuff
        wxBoxSizer *sizer_buttons = new wxBoxSizer(wxHORIZONTAL);

        frame = new wxFrame((wxFrame *)NULL, -1,  wxT("Map View RBO"), wxPoint(50,50), inital_size);
        m_menubar = new wxMenuBar;
        m_menu_view = new wxMenu;

        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Laser Scan"));
        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Robot"));
        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Particle"));
        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Path"));
        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Global Plan"));
        m_menu_view->AppendCheckItem(wxID_ANY,wxT("Local Plan"));
        m_menubar->Append(m_menu_view, wxT("&View"));
        //frame->SetMenuBar(m_menubar);
        wxImage inital_image = wxImage(inital_size.GetWidth(), inital_size.GetWidth());

        my_image = new MapScrolledWindow(frame, wxID_ANY, inital_image);
        // add the ros drawers
        my_image->Add(new RosLaserDrawer(node));
   		my_image->Add(new RosRobotDrawer(node));
   		m_drawer_goal = new RosGoalSelect(node,my_image);
   		my_image->Add(m_drawer_goal);
   		m_mouse_planner = new RosMousePlanner(node,my_image,"/move_base/TrajectoryPlannerROS/global_plan");
   		my_image->Add(m_mouse_planner);
   		my_image->Add(new RosPathDrawer(node,"/move_base/NavfnROS/plan",wxPen(*wxRED,1)));
   		my_image->Add(new RosPathDrawer(node,"/move_base/TrajectoryPlannerROS/global_plan",wxPen(*wxRED,3)));
   		my_image->Add(new RosPathDrawer(node,"/move_base/TrajectoryPlannerROS/local_plan",wxPen(*wxGREEN,1)));

		my_image->Connect(wxEVT_RIGHT_UP,wxMouseEventHandler(MapPlannerApp::OnRightMouse),NULL,this);

		// mouse menu
    	m_mouse_menu = new wxMenu();
    	wxMenuItem  *set_goal = m_mouse_menu->Append(wxID_ANY,wxT("Set Goal"));
    	this->Connect(set_goal->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapPlannerApp::OnMouseMenuGoal),NULL,this);
    	wxMenuItem  *set_center = m_mouse_menu->Append(wxID_ANY,wxT("Center Screen"));
    	this->Connect(set_center->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapPlannerApp::OnMouseMenuCenter),NULL,this);
    	wxMenuItem  *set_clear = m_mouse_menu->Append(wxID_ANY,wxT("Plan Path"));
    	this->Connect(set_clear->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapPlannerApp::OnMouseMenuPlan),NULL,this);
/*
        wxButton *button_zoom_in = new wxButton(frame, wxID_ZOOM_IN);
        sizer_buttons->Add(button_zoom_in);
        this->Connect(button_zoom_in->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomIn), NULL, my_image);
*/
        wxButton *button_zoom_100 = new wxButton(frame, wxID_ZOOM_100);
        this->Connect(button_zoom_100->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomReset), NULL, my_image);
        sizer_buttons->Add(button_zoom_100);

/*        wxButton *button_zoom_out = new wxButton(frame, wxID_ZOOM_OUT);
        this->Connect(button_zoom_out->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomOut), NULL, my_image);
        sizer_buttons->Add(button_zoom_out);
*/
        wxButton *button_cancel = new wxButton(frame, wxID_CANCEL);
        //this->Connect(button_cancle->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(), NULL, );
        sizer_buttons->Add(button_cancel);

        wxButton *button_map = new wxButton(frame, wxID_ANY,wxT("Request Map"));
        //this->Connect(button_cancle->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(), NULL, );
        sizer_buttons->Add(button_map);

        sizer_grid->Add(sizer_buttons);
        sizer_grid->Add(my_image, 1, wxEXPAND);
        sizer_grid->AddGrowableRow(1,1);
        sizer_grid->AddGrowableCol(0,1);

        sizer_box->Add(sizer_grid,1,wxEXPAND);
        frame->SetSizer(sizer_box);
        timer_ros = new wxTimer(this,wxID_ANY);
        timer_ros->Start(100,wxTIMER_CONTINUOUS);
        this->Connect(timer_ros->GetId(),wxEVT_TIMER, wxTimerEventHandler(MapPlannerApp::OnTimerRos), NULL, this);
        frame->Show();
        return true;
    }
private:
    void MapCallback(const nav_msgs::OccupancyGridConstPtr &mapMsg)
    {
    	//ROS_INFO("new map %ix%i\n",mapMsg->info.width, mapMsg->info.height);
    	my_image->SetMap(mapMsg);
    }

    void OnTimerRos(wxTimerEvent & event)
    {
    	bool update=true;
    	if(ros::getGlobalCallbackQueue()->isEmpty()) update=false;
    	ros::spinOnce();
    	if(update) my_image->Refresh();
    }

    void OnRightMouse(wxMouseEvent &event)
    {
    	m_right_mouse_click_pos = event.GetPosition();
    	my_image->PopupMenu(m_mouse_menu,event.GetPosition());
    }

    void OnMouseMenuGoal(wxEvent &event)
    {
    	m_drawer_goal->SetGoal(m_right_mouse_click_pos);
    }

    void OnMouseMenuCenter(wxEvent &event)
    {

    	my_image->SetCenter(my_image->mouse2map(m_right_mouse_click_pos));
    }

    void OnMouseMenuPlan(wxEvent &event)
    {
    	m_mouse_planner->SetStart(m_right_mouse_click_pos);
    }

};

IMPLEMENT_APP_NO_MAIN(MapPlannerApp)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_planner_rbo");
	return wxEntry(argc, argv);
}
