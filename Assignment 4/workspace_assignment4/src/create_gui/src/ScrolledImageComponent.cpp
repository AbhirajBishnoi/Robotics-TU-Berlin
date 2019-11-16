/*
 * ScrolledImageComponend.cpp
 *
 *  Created on: 01.10.2010
 *      Author: dermax
 */
#include "create_gui/ScrolledImageComponent.h"

    ScrolledImageComponent::ScrolledImageComponent(wxWindow* parent, wxWindowID id, wxString image_path) : wxScrolledWindow(parent, id)
    {
    	Init();
        wxImage image(image_path);
        if(!image.IsOk())
        {
            wxMessageBox(wxT("there was an error loading the image"));
            return;
        }
    	SetImage(image);
    }

    ScrolledImageComponent::ScrolledImageComponent(wxWindow* parent, wxWindowID id, wxImage &image) : wxScrolledWindow(parent, id)
    {
    	Init();
    	SetImage(image);
    }

    ScrolledImageComponent::~ScrolledImageComponent()
    {
        if(m_image) delete m_image;
        if(m_scaled_bitmap) delete m_scaled_bitmap;
    }

    void ScrolledImageComponent::Init()
    {
    	m_w=0;
    	m_h=0;
    	m_x0 = 0;
    	m_y0 = 0;
    	m_scaled_bitmap = 0;
    	m_image = 0;
    	m_scale = 1;
    	m_scale_step = 0.1;
    	this->Connect(wxEVT_MOUSEWHEEL,wxMouseEventHandler(ScrolledImageComponent::OnMouseWheel),NULL,this);
    	this->Connect(wxEVT_SCROLL_CHANGED,wxScrollEventHandler(ScrolledImageComponent::OnScroll),NULL,this);

    }

    void ScrolledImageComponent::SetImage(wxImage &image)
    {
    	if(m_w!=image.GetWidth() || m_h != image.GetHeight())
    	{
    		m_w = image.GetWidth();
    		m_h = image.GetHeight();
    		/* init scrolled area size, scrolling speed, etc. */
    		int scrolling_speed = 10;
    		SetScrollbars(scrolling_speed,scrolling_speed, m_w/scrolling_speed, m_h/scrolling_speed, 0, 0);
    	}
        if(m_image) delete m_image;
        if (m_scaled_bitmap!=NULL)
            delete m_scaled_bitmap;
        m_scaled_bitmap = 0;
        m_image = new wxImage(image);
//        this->Refresh();
    }
    void ScrolledImageComponent::SetImage(wxBitmap &image)
    {
    	if(m_w!=image.GetWidth() || m_h != image.GetHeight())
    	{
    		m_w = image.GetWidth();
    		m_h = image.GetHeight();
    		/* init scrolled area size, scrolling speed, etc. */
    		int scrolling_speed = 10;
    		SetScrollbars(scrolling_speed,scrolling_speed, m_w/scrolling_speed, m_h/scrolling_speed, 0, 0);
    	}
        if(m_image) delete m_image;
        if (m_scaled_bitmap!=NULL)
            delete m_scaled_bitmap;
        m_scaled_bitmap = 0;
        m_image = new wxImage(image.ConvertToImage());
 //       this->Refresh();
    }

    void ScrolledImageComponent::GetOffset(int *x, int *y)
    {
    	*x = 0;
    	*y = 0;
		if(m_scaled_bitmap)
		{
			int w_w,w_h;
			this->GetSize(&w_w, &w_h);
			*x = w_w/2 - m_scaled_bitmap->GetWidth()/2;
			*y = w_h/2 - m_scaled_bitmap->GetHeight()/2;
			if(*x<0) *x = 0;
			if(*y<0) *y = 0;
		}
    }

    void ScrolledImageComponent::DoDraw(wxDC& dc)
    {
    	if(m_image)
    	{
    		if(!m_scaled_bitmap)
    		{
    			rescale();
    		}
        /* render the image - in a real app, if your scrolled area
           is somewhat big, you will want to draw only visible parts,
           not everything like below */
    		int offset_x,offset_y;
    		GetOffset(&offset_x, &offset_y);
    		dc.Clear();
    		dc.DrawBitmap(*m_scaled_bitmap, offset_x, offset_y, false);
    	}
    }


    void ScrolledImageComponent::OnDraw(wxDC& dc)
    {
    	DoDraw(dc);
    }

    void ScrolledImageComponent::rescale()
    {
        if (m_scaled_bitmap!=NULL)
            delete m_scaled_bitmap;
        if(m_scale<=0) m_scale = 1;
        if(m_scale>=100) m_scale = 100;

        wxImage scaledimg=m_image->Scale((m_w*m_scale),(m_h*m_scale));
        m_scaled_bitmap=new wxBitmap(scaledimg);

        SetVirtualSize((m_w*m_scale),(m_h*m_scale));
   }
    void ScrolledImageComponent::SetScale(double scale)
    {
    	if((m_scale!=scale) && (m_scaled_bitmap!=NULL))
    	{
    		delete m_scaled_bitmap;
    		m_scaled_bitmap=NULL;
            if(scale>0.05)	m_scale = scale;
            this->Refresh();
    	}
    }

    void ScrolledImageComponent::SetCenter(wxPoint p)
    {
    	SetCenter(p.x,p.y);
    }
    void ScrolledImageComponent::SetCenter(int x ,int y)
    {
    	printf("center(%i,%i)\n",x,y);
    	int ppx,ppy;
    	GetScrollPixelsPerUnit(&ppx,&ppy);
    	wxSize size = this->GetSize();

    	Scroll((x - size.x/2)/ppx, (y - size.y/2)/ppy);
    }

    void ScrolledImageComponent::OnMouseWheel(wxMouseEvent & event)
    {
    	wxPoint pos_on_screen = event.GetPosition();
    	wxPoint pos_on_map;
    	CalcUnscrolledPosition(pos_on_screen.x, pos_on_screen.y, &pos_on_map.x, &pos_on_map.y);
    	double x = pos_on_map.x/m_scale;
    	double y = pos_on_map.y/m_scale;
   		SetScale(m_scale + m_scale_step*event.m_wheelRotation/(double)event.m_wheelDelta);
    	pos_on_map.x = x*m_scale;
    	pos_on_map.y = y*m_scale;

    	int ppx,ppy;
    	GetScrollPixelsPerUnit(&ppx,&ppy);
    	Scroll((pos_on_map.x - pos_on_screen.x)/ppx, (pos_on_map.y - pos_on_screen.y)/ppy);
    }

    void ScrolledImageComponent::OnScroll(wxScrollEvent & event)
    {

    }

    wxPoint ScrolledImageComponent::mouse2map(wxPoint p)
    {
    	wxPoint map_p;
    	p=CalcUnscrolledPosition(p);
		int offset_x,offset_y;
		GetOffset(&offset_x, &offset_y);
    	map_p.x = (p.x-offset_x);
    	map_p.y = (p.y-offset_y);
    	return map_p;
    }

