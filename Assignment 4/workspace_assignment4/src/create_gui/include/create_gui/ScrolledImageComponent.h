/*
 * ScrolledImageComponent.h
 *
 *  Created on: 01.10.2010
 *      Author: dermax
 */

#ifndef SCROLLEDIMAGECOMPONENT_H_
#define SCROLLEDIMAGECOMPONENT_H_

#include "wx/wx.h"

class ScrolledImageComponent : public wxScrolledWindow
{
protected:
    wxImage *m_image;
    wxBitmap *m_scaled_bitmap;
    int  m_w,m_h;
    int m_x0,m_y0;
    double m_scale;
    double m_scale_step;

    void Init();
    void rescale();
    void OnMouseWheel(wxMouseEvent & event);
    void GetOffset(int *x, int *y);
public:
    wxPoint mouse2map(wxPoint p);
    void SetCenter(wxPoint p);
    void SetCenter(int x ,int y);
    ScrolledImageComponent(wxWindow* parent, wxWindowID id, wxString image_path);
    ScrolledImageComponent(wxWindow* parent, wxWindowID id, wxImage &image);
    ~ScrolledImageComponent();
    void SetImage(wxImage &image);
    void SetImage(wxBitmap &image);
    void DoDraw(wxDC& dc);
    void OnDraw(wxDC& dc);
    void SetScale(double scale);
    double GetScale() {return m_scale;};
    void ZoomIn() {	SetScale(m_scale + m_scale_step);};
    void wxEvtZoomIn(wxEvent & event) {ZoomIn();};
    void ZoomReset() {	SetScale(1.0);};
    void wxEvtZoomReset(wxEvent & event) {ZoomReset();};
    void ZoomOut(){	SetScale(m_scale - m_scale_step);};
    void wxEvtZoomOut(wxEvent & event) {ZoomOut();};
    void OnScroll(wxScrollEvent & event);


};

#endif /* SCROLLEDIMAGECOMPONENT_H_ */
