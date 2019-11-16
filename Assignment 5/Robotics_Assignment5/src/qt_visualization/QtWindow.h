//
// Copyright (c) 2009, Markus Rickert, edited by Peter Lehner
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Technische Universitaet Muenchen nor the names of
//   its contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef _QTWINDOW_H_
#define _QTWINDOW_H_

#include <QAction>
#include <QDockWidget>
#include <QGraphicsView>
#include <QMainWindow>
#include <QMutex>
#include <QTableView>
#include <boost/shared_ptr.hpp>
#include <rl/kin/Kinematics.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/WorkspaceSphereExplorer.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/bullet/Model.h>
#include <rl/sg/so/Scene.h>
#include <rl/sg/bullet/Scene.h>
#include "../TutorialPlanSystem.h"
#include "QtViewer.h"

#include "QtPlanningThread.h"

class QtWindow : public QMainWindow
{
	Q_OBJECT
	
public:
        QtWindow(TutorialPlanSystem* system, QWidget* parent = NULL, Qt::WFlags f = 0);

        virtual ~QtWindow();
	
        static QtWindow* instance(TutorialPlanSystem* system);
	
        boost::shared_ptr< rl::plan::Model > model; //model for display

        boost::shared_ptr<rl::sg::so::Scene> scene;

        QtViewer* viewer;
	
public slots:
	
	void getGoalConfiguration();
	
	void getRandomConfiguration();
	
	void getRandomFreeConfiguration();
	
	void getStartConfiguration();
	
	void open();
	
    void startPlanning();

	void reset();
	
	void saveImage();
	
	void saveScene();
	
	void setGoalConfiguration();
	
	void setStartConfiguration();
	
	void toggleCamera();
	
	void toggleConfiguration();
	
	void toggleView(const bool& doOn);
	
protected:
        QtWindow(QWidget* parent = NULL, Qt::WindowFlags f = 0);
	
private:
	void connect(const QObject* sender, const QObject* receiver);
	
	void disconnect(const QObject* sender, const QObject* receiver);
	
	void init();
	
        void load();
	
	QDockWidget* configurationDockWidget;
	
	QTableView* configurationView;
	
	QAction* exitAction;
	
	QAction* getGoalConfigurationAction;
	
	QAction* getRandomConfigurationAction;
	
	QAction* getRandomFreeConfigurationAction;
	
	QAction* getStartConfigurationAction;
	
	QDockWidget* plannerDockWidget;
	
	QTableView* plannerView;
	
	QAction* resetAction;
	
	QAction* saveImageAction;
	
	QAction* saveSceneAction;
	
	QAction* setGoalConfigurationAction;
	
	QAction* setStartConfigurationAction;
	
        static QtWindow* singleton;
	
        QAction* startPlanningAction;
	
	QAction* toggleCameraAction;
	
	QAction* toggleConfigurationAction;
	
	QAction* toggleConfigurationEdgesAction;
	
	QAction* toggleConfigurationVerticesAction;
	
	QAction* togglePlannerAction;
	
	QAction* toggleViewAction;

        QtPlanningThread* planningThread;

        TutorialPlanSystem* system;

};

#endif // _MAINWINDOW_H_
