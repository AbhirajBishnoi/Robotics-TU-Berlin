#include "ros/ros.h"
#include <wx/wx.h>

#include "create_gui/create_move.h"
#include "create_gui/create_status.h"
#include "icons/go-down.xpm"
#include "icons/go-up.xpm"
#include "icons/rotate-left.xpm"
#include "icons/rotate-right.xpm"

class CreateButtonMoveWX : public wxFrame
{
	CreateMove* create_move;
	CreateStatus* create_status;
	wxStaticText *txt_bat;
	wxStaticText *txt_speed;
	wxTimer	*timer_status;
	int speed;
public:
	CreateButtonMoveWX(const wxString& title);

    void OnLeft(wxEvent& event);
    void OnRight(wxEvent & event);
    void OnForward(wxEvent & event);
    void OnBackward(wxEvent & event);
    void OnStop(wxEvent & event);
    void OnCloseWindow(wxEvent & event);
    void OnSlider(wxCommandEvent & event);
    void OnTimerStatus(wxTimerEvent & event);
};

CreateButtonMoveWX::CreateButtonMoveWX(const wxString& title)
       : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(220, 180))
{
	create_move = new CreateMove();
	create_status = new CreateStatus();
	this->speed = 50;

  //wxPanel *panel = new wxPanel(this, wxID_ANY);
  wxGridSizer *grid = new wxGridSizer(3, 3,2,2);
  wxBoxSizer *boxsizer = new wxBoxSizer(wxVERTICAL);
  wxFlexGridSizer *sizer = new  wxFlexGridSizer(2,1,0,0);

  wxBitmapButton *button_left = new wxBitmapButton(this,wxID_ANY,wxBitmap(rotate_left_xpm));
  button_left->Connect( wxEVT_LEFT_DOWN, wxEventHandler(CreateButtonMoveWX::OnLeft), NULL, this );
  button_left->Connect( wxEVT_LEFT_UP, wxEventHandler(CreateButtonMoveWX::OnStop), NULL, this );

  wxBitmapButton *button_right = new wxBitmapButton(this, wxID_ANY, wxBitmap(rotate_right_xpm));
  button_right->Connect( wxEVT_LEFT_DOWN, wxEventHandler(CreateButtonMoveWX::OnRight), NULL, this );
  button_right->Connect( wxEVT_LEFT_UP, wxEventHandler(CreateButtonMoveWX::OnStop), NULL, this );

  wxBitmapButton *button_forward = new wxBitmapButton(this,wxID_ANY,wxBitmap(go_up_xpm));
  button_forward->Connect( wxEVT_LEFT_DOWN, wxEventHandler(CreateButtonMoveWX::OnForward), NULL, this );
  button_forward->Connect( wxEVT_LEFT_UP, wxEventHandler(CreateButtonMoveWX::OnStop), NULL, this );


  wxBitmapButton *button_backward = new wxBitmapButton(this, wxID_ANY,wxBitmap(go_down_xpm));
  button_backward->Connect( wxEVT_LEFT_DOWN, wxEventHandler(CreateButtonMoveWX::OnBackward), NULL, this );
  button_backward->Connect( wxEVT_LEFT_UP, wxEventHandler(CreateButtonMoveWX::OnStop), NULL, this );

  wxButton *button_stop = new wxButton(this, wxID_STOP,wxT("Stop"));
  button_stop->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxEventHandler(CreateButtonMoveWX::OnStop), NULL, this);
  button_stop->SetFocus();

  wxSlider *slider_speed = new wxSlider(this, wxID_ANY, speed,1,1000);
  slider_speed->Connect(wxEVT_COMMAND_SLIDER_UPDATED, wxCommandEventHandler(CreateButtonMoveWX::OnSlider), NULL, this);

  slider_speed->SetLabel(wxT("Geschwindigkeit"));
  // the sizer for the toppart with the slider
  wxBoxSizer *sizer_top = new wxBoxSizer(wxHORIZONTAL);
  sizer_top->Add(new wxStaticText(this,wxID_ANY,wxT("Speed:")));
  sizer_top->Add(slider_speed, 1, wxEXPAND);
  txt_speed = new wxStaticText(this,wxID_ANY,wxT("0000"));
  sizer_top->Add(txt_speed);

  txt_bat = new wxStaticText(this, -1, wxT(""));
  timer_status = new wxTimer(this,wxID_ANY);
  this->Connect(timer_status->GetId(),wxEVT_TIMER, wxTimerEventHandler(CreateButtonMoveWX::OnTimerStatus), NULL, this);

  grid->Add(txt_bat, 0, wxEXPAND);
  grid->Add(button_forward, 0, wxEXPAND);
  grid->Add(new wxStaticText(this, -1, wxT("")), 0, wxEXPAND);
  grid->Add(button_left, 0, wxEXPAND);
  grid->Add(button_stop, 0, wxEXPAND);
  grid->Add(button_right, 0, wxEXPAND);
  grid->Add(new wxStaticText(this, -1, wxT("")), 0, wxEXPAND);
  grid->Add(button_backward, 0, wxEXPAND);
  grid->Add(new wxStaticText(this, -1, wxT("")), 0, wxEXPAND);
  sizer->Add(sizer_top,0,wxEXPAND);
  sizer->Add(grid,1,wxEXPAND);
  //sizer->AddGrowableRow(2, 1);
  //sizer->AddGrowableCol(2, 1);

  boxsizer->Add(sizer,1, wxEXPAND);
  SetSizer(boxsizer);
  Centre();
  timer_status->Start(100,wxTIMER_CONTINUOUS);

}

void CreateButtonMoveWX::OnLeft(wxEvent & event)
{
	create_move->move(this->speed,-this->speed);
}

void CreateButtonMoveWX::OnRight(wxEvent & WXUNUSED(event))
{
	create_move->move(-this->speed,this->speed);
}

void CreateButtonMoveWX::OnForward(wxEvent & WXUNUSED(event))
{
	create_move->move(this->speed,this->speed);
}

void CreateButtonMoveWX::OnBackward(wxEvent & WXUNUSED(event))
{
	create_move->move(-this->speed,-this->speed);
}

void CreateButtonMoveWX::OnStop(wxEvent & WXUNUSED(event))
{
	create_move->stop();
}

void CreateButtonMoveWX::OnCloseWindow(wxEvent & event)
{
	OnStop(event);
	this->Close();
}

void CreateButtonMoveWX::OnSlider(wxCommandEvent & event)
{
	this->speed = event.GetInt();
	wxString str;
	str.Printf(wxT("%4i"),speed);
	this->txt_speed->SetLabel(str);
}

void CreateButtonMoveWX::OnTimerStatus(wxTimerEvent & event)
{
	wxString str;
	str.Printf(wxT("%2.2fV"),create_status->getBatStatus());
	this->txt_bat->SetLabel(str);
	str.Printf(wxT("%4i"),speed);
	this->txt_speed->SetLabel(str);
}


class CreateButtonMoveApp : public wxApp
{
	CreateButtonMoveWX *btnapp;
  public:
    virtual bool OnInit();
	int FilterEvent(wxEvent& event);
};

int CreateButtonMoveApp::FilterEvent(wxEvent& event)
{
	if((event.GetEventType()==wxEVT_KEY_DOWN) || ( event.GetEventType()==wxEVT_KEY_UP))
	{
		if (( ((wxKeyEvent&)event).GetKeyCode()==WXK_UP) && wxGetKeyState(WXK_UP))
		{
			btnapp->OnForward(event);
			return true;
		}
		if (( ((wxKeyEvent&)event).GetKeyCode()==WXK_DOWN) && wxGetKeyState(WXK_DOWN))
		{
			btnapp->OnBackward(event);
			return true;
		}
		if (( ((wxKeyEvent&)event).GetKeyCode()==WXK_LEFT) && wxGetKeyState(WXK_LEFT))
		{
			btnapp->OnLeft(event);
			return true;
		}
		if (( ((wxKeyEvent&)event).GetKeyCode()==WXK_RIGHT) && wxGetKeyState(WXK_RIGHT))
		{
			btnapp->OnRight(event);
			return true;
		}
	}
	if ( event.GetEventType()==wxEVT_KEY_UP)
	{
		btnapp->OnStop(event);
		return true;
	}
	ros::spinOnce();
    return -1;
}


bool CreateButtonMoveApp::OnInit()
{
	btnapp = new CreateButtonMoveWX(wxT("Create Button Move"));
    btnapp->Show(true);

    return true;
}

IMPLEMENT_APP_NO_MAIN(CreateButtonMoveApp)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "create_move_wx");
	return wxEntry(argc, argv);
}


