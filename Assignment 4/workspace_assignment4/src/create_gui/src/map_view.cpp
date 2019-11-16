#include "wx/wx.h"
#include <iostream>
#include <string>

#include "std_srvs/Empty.h"
#include "localization/SetInitialPose.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include "create_gui/MapScrolledWindow.h"
#include "create_gui/RosRobotDrawer.h"
#include "create_gui/RosRobotPathDrawer.h"
#include "create_gui/RosLaserDrawer.h"
#include "create_gui/RosPathDrawer.h"
#include "create_gui/RosGoalSelect.h"
#include "create_gui/RosPoseDrawer.h"
#include "create_gui/RosPoseArrayDrawer.h"
#include "create_gui/RosPointCloudDrawer.h"

class MapViewApp: public wxApp
{
    wxFrame *frame;
    MapScrolledWindow* my_image;
    ros::Subscriber map_subscriber;
    ros::NodeHandle node;
	wxTimer	*timer_ros;
	wxMenu *m_mouse_menu;
	RosGoalSelect *m_drawer_goal;
	wxPoint m_right_mouse_click_pos;
	wxMenuBar *m_menubar;
    wxMenu *m_menu_view;
    wxCheckBox *m_check_likelihood;
    RosRobotPathDrawer *m_drawer_robot_path;
public:

    bool OnInit()
    {
    	// subscribe to the map topic
    	map_subscriber = node.subscribe("/map", 1, &MapViewApp::MapCallback, this);

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
//        my_image->Add(new RosPointCloudDrawer(node,"/particle_filter/likelihood_field"));
   		m_drawer_goal = new RosGoalSelect(node,my_image);
   		my_image->Add(m_drawer_goal);
   		my_image->Add(new RosPathDrawer(node,"/move_base/NavfnROS/plan",wxPen(*wxRED,1)));
   		my_image->Add(new RosPathDrawer(node,"/move_base/TrajectoryPlannerROS/global_plan",wxPen(*wxRED,3)));
   		my_image->Add(new RosPathDrawer(node,"/move_base/TrajectoryPlannerROS/local_plan",wxPen(*wxGREEN,1)));
   		my_image->Add(new RosPoseArrayDrawer(node,"/particle_filter/particlecloud"));
   		m_drawer_robot_path = new RosRobotPathDrawer(node,"/base_link");
   		my_image->Add(m_drawer_robot_path);
   		my_image->Add(new RosRobotDrawer(node));
   		my_image->Add(new RosPoseDrawer(node,"/particle_filter/estimated_pose"));

		my_image->Connect(wxEVT_RIGHT_UP,wxMouseEventHandler(MapViewApp::OnRightMouse),NULL,this);

		// mouse menu
    	m_mouse_menu = new wxMenu();
    	wxMenuItem  *set_goal = m_mouse_menu->Append(wxID_ANY,wxT("set goal"));
    	this->Connect(set_goal->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapViewApp::OnMouseMenuGoal),NULL,this);
    	wxMenuItem  *clear_path = m_mouse_menu->Append(wxID_ANY,wxT("clear path"));
    	this->Connect(clear_path->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapViewApp::OnMouseMenuClear),NULL,this);
    	wxMenuItem  *set_clear = m_mouse_menu->Append(wxID_ANY,wxT("distribute normal"));
    	this->Connect(set_clear->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxEventHandler(MapViewApp::OnMouseMenuNormal),NULL,this);
/*
        wxButton *button_zoom_in = new wxButton(frame, wxID_ZOOM_IN);
        sizer_buttons->Add(button_zoom_in);
        this->Connect(button_zoom_in->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomIn), NULL, my_image);
*/
        wxButton *button_zoom_100 = new wxButton(frame, wxID_ZOOM_100);
        this->Connect(button_zoom_100->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomReset), NULL, my_image);
        sizer_buttons->Add(button_zoom_100);

        m_check_likelihood = new wxCheckBox(frame, wxID_ANY, wxT("show likelihood field"));
        this->Connect(m_check_likelihood->GetId(),wxEVT_COMMAND_CHECKBOX_CLICKED, wxEventHandler(MapViewApp::OnLikelihood), NULL, this);
        sizer_buttons->Add(m_check_likelihood);

/*        wxButton *button_zoom_out = new wxButton(frame, wxID_ZOOM_OUT);
        this->Connect(button_zoom_out->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapScrolledWindow::wxEvtZoomOut), NULL, my_image);
        sizer_buttons->Add(button_zoom_out);
*/
        wxButton *button_distribute = new wxButton(frame, wxID_ANY, wxT("distribute uniform"));
        this->Connect(button_distribute->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapViewApp::OnUniform), NULL, this);
        sizer_buttons->Add(button_distribute);

        wxButton *button_publish = new wxButton(frame, wxID_ANY, wxT("publish all"));
        this->Connect(button_publish->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(MapViewApp::OnPublishAll), NULL, this);
        sizer_buttons->Add(button_publish);

        //wxButton *button_map = new wxButton(frame, wxID_ANY,wxT("Request Map"));
        //this->Connect(button_cancle->GetId(),wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(), NULL, );
        //sizer_buttons->Add(button_map);

        sizer_grid->Add(sizer_buttons);
        sizer_grid->Add(my_image, 1, wxEXPAND);
        sizer_grid->AddGrowableRow(1,1);
        sizer_grid->AddGrowableCol(0,1);

        sizer_box->Add(sizer_grid,1,wxEXPAND);
        frame->SetSizer(sizer_box);
        timer_ros = new wxTimer(this,wxID_ANY);
        timer_ros->Start(500,wxTIMER_CONTINUOUS);
        this->Connect(timer_ros->GetId(),wxEVT_TIMER, wxTimerEventHandler(MapViewApp::OnTimerRos), NULL, this);
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

    void OnUniform(wxEvent &event)
    {
    	ros::ServiceClient service_uniform = node.serviceClient<std_srvs::Empty>("/particle_filter/distribute_uniform");
    	if(service_uniform.waitForExistence(ros::Duration(5)))
    	{
    		std_srvs::Empty e;
    		service_uniform.call(e);
    	} else
    	{
    		ROS_ERROR("faild to call service");
    	}
    }

    void OnPublishAll(wxEvent &event)
    {
    	ros::ServiceClient service_uniform = node.serviceClient<std_srvs::Empty>("/particle_filter/publish_everything");
    	if(service_uniform.waitForExistence(ros::Duration(5)))
    	{
    		std_srvs::Empty e;
    		service_uniform.call(e);
    	} else
    	{
    		ROS_ERROR("faild to call service");
    	}
    }

    void OnLikelihood(wxEvent &event)
    {
    	if(m_check_likelihood->GetValue())
    	{
        	// subscribe to the map topic
        	map_subscriber = node.subscribe("/particle_filter/likelihood_map", 1, &MapViewApp::MapCallback, this);
        	ros::ServiceClient service_uniform = node.serviceClient<std_srvs::Empty>("/particle_filter/publish_everything");
        	if(service_uniform.waitForExistence(ros::Duration(5)))
        	{
        		std_srvs::Empty e;
        		service_uniform.call(e);
        	} else
        	{
        		ROS_ERROR("faild to call service");
        	}

    	} else
    	{
    		// get map via map_server
    		nav_msgs::GetMap::Request req;
    		nav_msgs::GetMap::Response resp;
        	// subscribe to the map topic
        	map_subscriber = node.subscribe("/map", 1, &MapViewApp::MapCallback, this);
        	if (!ros::service::call("static_map", req, resp)) {
        		ROS_WARN("Request for map failed");
        	} else
        	{
        		boost::shared_ptr< ::nav_msgs::OccupancyGrid const> m(new nav_msgs::OccupancyGrid(resp.map));
        		MapCallback(m);
        	}
    	}
    }

    void OnMouseMenuNormal(wxEvent &event)
    {
    	ros::ServiceClient service_uniform = node.serviceClient<localization::SetInitialPose>("/particle_filter/distribute_normal");
    	if(service_uniform.waitForExistence(ros::Duration(5)))
    	{
    		wxPoint p = my_image->mouse2map(m_right_mouse_click_pos);
    		double x=0,y=0;
    		my_image->toMap(p.x,p.y,&x,&y);
    		printf("distribute normal x=%f, y=%f\n",x,y);
    		localization::SetInitialPose pose;
    		pose.request.var_x = 0.2;
    		pose.request.var_y = 0.2;
    		pose.request.var_theta = 1.6;
    		pose.request.x = x;
    		pose.request.y = y;
    		pose.request.theta = 0;
    		service_uniform.call(pose);
    	} else
    	{
    		ROS_ERROR("faild to call service");
    	}
    	my_image->Refresh();

    }
    void OnMouseMenuClear(wxEvent &event)
    {
    	m_drawer_robot_path->ClearPath();
    }

};

IMPLEMENT_APP_NO_MAIN(MapViewApp)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_view_rbo");
	return wxEntry(argc, argv);
}
