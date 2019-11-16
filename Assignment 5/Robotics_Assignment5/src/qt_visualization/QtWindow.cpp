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

#include <QApplication>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QGLWidget>
#include <QGraphicsView>
#include <QHeaderView>
#include <QLayout>
#include <QMenuBar>
#include <QMutexLocker>
#include <QPainter>
#include <QPrinter>
#include <QMessageBox>
#include <QMetaType>
#include <Inventor/nodes/SoCamera.h>
#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/Qt/SoQt.h>
#include "rl/math/Unit.h"
#include "rl/math/Rotation.h"

#include "QtWindow.h"

QtWindow::QtWindow(TutorialPlanSystem* system, QWidget* parent, Qt::WFlags f):
    QMainWindow(parent, f),
    configurationDockWidget(new QDockWidget(this)),
    configurationView(new QTableView(this)),
    exitAction(new QAction(this)),
    getGoalConfigurationAction(new QAction(this)),
    getRandomConfigurationAction(new QAction(this)),
    getRandomFreeConfigurationAction(new QAction(this)),
    getStartConfigurationAction(new QAction(this)),
    plannerDockWidget(new QDockWidget(this)),
    plannerView(new QTableView(this)),
    resetAction(new QAction(this)),
    saveImageAction(new QAction(this)),
    saveSceneAction(new QAction(this)),
    setGoalConfigurationAction(new QAction(this)),
    setStartConfigurationAction(new QAction(this)),
    startPlanningAction(new QAction(this)),
    toggleCameraAction(new QAction(this)),
    toggleConfigurationAction(new QAction(this)),
    toggleConfigurationEdgesAction(new QAction(this)),
    toggleConfigurationVerticesAction(new QAction(this)),
    togglePlannerAction(new QAction(this)),
    toggleViewAction(new QAction(this)),
    planningThread()
{
    QtWindow::singleton = this;

    this->resize(640, 480);

    this->system = system;

    this->viewer = new QtViewer();
    this->setCentralWidget(this->viewer);

    this->planningThread= new QtPlanningThread(new QMutex(QMutex::Recursive), this->system, viewer);

    qRegisterMetaType< rl::math::Real >("rl::math::Real");
    qRegisterMetaType< rl::math::Transform >("rl::math::Transform");
    qRegisterMetaType< rl::math::Vector >("rl::math::Vector");
    qRegisterMetaType< rl::plan::VectorList >("rl::plan::VectorList");

    this->init();
    this->load();
}

QtWindow::~QtWindow()
{	
	this->reset();
    QtWindow::singleton = NULL;
}

void
QtWindow::connect(const QObject* sender, const QObject* receiver)
{
    QObject::connect(
            sender,
            SIGNAL(configurationRequested(const rl::math::Vector&)),
            receiver,
            SLOT(drawConfiguration(const rl::math::Vector&))
    );

    QObject::connect(
            sender,
            SIGNAL(configurationEdgeRequested(const rl::math::Vector&, const rl::math::Vector&, const bool&)),
            receiver,
            SLOT(drawConfigurationEdge(const rl::math::Vector&, const rl::math::Vector&, const bool&))
    );

    QObject::connect(
            sender,
            SIGNAL(configurationPathRequested(const rl::plan::VectorList&)),
            receiver,
            SLOT(drawConfigurationPath(const rl::plan::VectorList&))
    );

    QObject::connect(
            sender,
            SIGNAL(configurationVertexRequested(const rl::math::Vector&, const bool&)),
            receiver,
            SLOT(drawConfigurationVertex(const rl::math::Vector&, const bool&))
    );

    QObject::connect(
            sender,
            SIGNAL(edgeResetRequested()),
            receiver,
            SLOT(resetEdges())
    );

    QObject::connect(
            sender,
            SIGNAL(lineRequested(const rl::math::Vector&, const rl::math::Vector&)),
            receiver,
            SLOT(drawLine(const rl::math::Vector&, const rl::math::Vector&))
    );

    QObject::connect(
            sender,
            SIGNAL(lineResetRequested()),
            receiver,
            SLOT(resetLines())
    );

    QObject::connect(
            sender,
            SIGNAL(resetRequested()),
            receiver,
            SLOT(reset())
    );

    QObject::connect(
            sender,
            SIGNAL(sphereRequested(const rl::math::Vector&, const rl::math::Real&)),
            receiver,
            SLOT(drawSphere(const rl::math::Vector&, const rl::math::Real&))
    );

    QObject::connect(
            sender,
            SIGNAL(sweptVolumeRequested(const rl::plan::VectorList&)),
            receiver,
            SLOT(drawSweptVolume(const rl::plan::VectorList&))
    );

    QObject::connect(
            sender,
            SIGNAL(vertexResetRequested()),
            receiver,
            SLOT(resetVertices())
    );

    QObject::connect(
            sender,
            SIGNAL(workRequested(const rl::math::Transform&)),
            receiver,
            SLOT(drawWork(const rl::math::Transform&))
    );

    QObject::connect(
            sender,
            SIGNAL(workEdgeRequested(const rl::math::Vector&, const rl::math::Vector&)),
            receiver,
            SLOT(drawWorkEdge(const rl::math::Vector&, const rl::math::Vector&))
    );

    QObject::connect(
            sender,
            SIGNAL(workPathRequested(const rl::plan::VectorList&)),
            receiver,
            SLOT(drawWorkPath(const rl::plan::VectorList&))
    );
}

void
QtWindow::disconnect(const QObject* sender, const QObject* receiver)
{
    QObject::disconnect(sender, NULL, receiver, NULL);
}

void
QtWindow::getGoalConfiguration()
{
    this->system->setConfiguration(this->system->getGoalConfiguration());
    this->viewer->drawConfiguration(this->system->getConfiguration());
}

void
QtWindow::getRandomConfiguration()
{
    rl::math::Vector config(this->model->kin->getDof());
    this->system->getRandomConfiguration(config);
    this->system->setConfiguration(config);
    this->viewer->drawConfiguration(config);
}

void
QtWindow::getRandomFreeConfiguration()
{
    rl::math::Vector config(this->model->kin->getDof());
    this->system->getRandomFreeConfiguration(config);
    this->system->setConfiguration(config);
    this->viewer->drawConfiguration(config);
}

void
QtWindow::getStartConfiguration()
{
    this->system->setConfiguration(this->system->getStartConfiguration());
    this->viewer->drawConfiguration(this->system->getConfiguration());
}

void
QtWindow::init()
{
    QMenu* fileMenu = this->menuBar()->addMenu("File");

    this->saveImageAction->setText("Save as PNG");
    this->saveImageAction->setShortcut(QKeySequence("Return"));
    QObject::connect(this->saveImageAction, SIGNAL(triggered()), this, SLOT(saveImage()));
    this->addAction(this->saveImageAction);
    fileMenu->addAction(this->saveImageAction);

    this->saveSceneAction->setText("Save as VRML");
    this->saveSceneAction->setShortcut(QKeySequence("Ctrl+Return"));
    QObject::connect(this->saveSceneAction, SIGNAL(triggered()), this, SLOT(saveScene()));
    this->addAction(this->saveSceneAction);
    fileMenu->addAction(this->saveSceneAction);

    fileMenu->addSeparator();

    this->exitAction->setText("Exit");
    QObject::connect(this->exitAction, SIGNAL(triggered()), qApp, SLOT(quit()));
    this->addAction(this->exitAction);
    fileMenu->addAction(this->exitAction);

    QMenu* configurationMenu = this->menuBar()->addMenu("Configuration");

    this->getRandomConfigurationAction->setText("Random");
    this->getRandomConfigurationAction->setShortcut(QKeySequence("F3"));
    QObject::connect(this->getRandomConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomConfiguration()));
    this->addAction(this->getRandomConfigurationAction);
    configurationMenu->addAction(this->getRandomConfigurationAction);

    this->getRandomFreeConfigurationAction->setText("Random (Collision-Free)");
    this->getRandomFreeConfigurationAction->setShortcut(QKeySequence("F4"));
    QObject::connect(this->getRandomFreeConfigurationAction, SIGNAL(triggered()), this, SLOT(getRandomFreeConfiguration()));
    this->addAction(this->getRandomFreeConfigurationAction);
    configurationMenu->addAction(this->getRandomFreeConfigurationAction);

    QMenu* plannerMenu = this->menuBar()->addMenu("Planner");

    this->getStartConfigurationAction->setText("Get Start Configuration");
    this->getStartConfigurationAction->setShortcut(QKeySequence("F1"));
    QObject::connect(this->getStartConfigurationAction, SIGNAL(triggered()), this, SLOT(getStartConfiguration()));
    this->addAction(this->getStartConfigurationAction);
    plannerMenu->addAction(this->getStartConfigurationAction);

    this->setStartConfigurationAction->setText("Set Start Configuration");
    this->setStartConfigurationAction->setShortcut(QKeySequence("CTRL+F1"));
    QObject::connect(this->setStartConfigurationAction, SIGNAL(triggered()), this, SLOT(setStartConfiguration()));
    this->addAction(this->setStartConfigurationAction);
    plannerMenu->addAction(this->setStartConfigurationAction);

    plannerMenu->addSeparator();

    this->getGoalConfigurationAction->setText("Get Goal Configuration");
    this->getGoalConfigurationAction->setShortcut(QKeySequence("F2"));
    QObject::connect(this->getGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(getGoalConfiguration()));
    this->addAction(this->getGoalConfigurationAction);
    plannerMenu->addAction(this->getGoalConfigurationAction);

    this->setGoalConfigurationAction->setText("Set Goal Configuration");
    this->setGoalConfigurationAction->setShortcut(QKeySequence("CTRL+F2"));
    QObject::connect(this->setGoalConfigurationAction, SIGNAL(triggered()), this, SLOT(setGoalConfiguration()));
    this->addAction(this->setGoalConfigurationAction);
    plannerMenu->addAction(this->setGoalConfigurationAction);

    plannerMenu->addSeparator();

    this->startPlanningAction->setText("Start Planner");
    this->startPlanningAction->setShortcut(QKeySequence("Space"));
    QObject::connect(this->startPlanningAction, SIGNAL(triggered()), this, SLOT(startPlanning()));
    this->addAction(this->startPlanningAction);
    plannerMenu->addAction(this->startPlanningAction);

    this->resetAction->setText("Reset");
    this->resetAction->setShortcut(QKeySequence("F12"));
    QObject::connect(this->resetAction, SIGNAL(triggered()), this, SLOT(reset()));
    this->addAction(this->resetAction);
    plannerMenu->addAction(this->resetAction);

    QMenu* viewMenu = this->menuBar()->addMenu("View");

    this->toggleViewAction->setCheckable(true);
    this->toggleViewAction->setText("Active");
    QObject::connect(this->toggleViewAction, SIGNAL(toggled(bool)), this, SLOT(toggleView(bool)));
    this->addAction(this->toggleViewAction);
    viewMenu->addAction(this->toggleViewAction);

    viewMenu->addSeparator();

    this->toggleConfigurationEdgesAction->setCheckable(true);
    this->toggleConfigurationEdgesAction->setChecked(true);
    this->toggleConfigurationEdgesAction->setText("Configuration Edges");
    QObject::connect(this->toggleConfigurationEdgesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationEdges(bool)));
    this->addAction(this->toggleConfigurationEdgesAction);
    viewMenu->addAction(this->toggleConfigurationEdgesAction);

    this->toggleConfigurationVerticesAction->setCheckable(true);
    this->toggleConfigurationVerticesAction->setChecked(false);
    this->toggleConfigurationVerticesAction->setText("Configuration Vertices");
    QObject::connect(this->toggleConfigurationVerticesAction, SIGNAL(toggled(bool)), this->viewer, SLOT(toggleConfigurationVertices(bool)));
    this->addAction(this->toggleConfigurationVerticesAction);
    viewMenu->addAction(this->toggleConfigurationVerticesAction);

    viewMenu->addSeparator();

    this->toggleCameraAction->setText("Perspective/Orthographic");
    this->toggleCameraAction->setShortcut(QKeySequence("F9"));
    QObject::connect(this->toggleCameraAction, SIGNAL(triggered()), this, SLOT(toggleCamera()));
    this->addAction(this->toggleCameraAction);
    viewMenu->addAction(this->toggleCameraAction);
}

QtWindow*
QtWindow::instance(TutorialPlanSystem* system)
{
    if (NULL == QtWindow::singleton)
    {
        new QtWindow(system);
    }

    return QtWindow::singleton;
}

void
QtWindow::load()
{

    scene.reset(new rl::sg::so::Scene());
    scene->load("../xml/rlsg/unimation-puma560-rbo_wall.xml");
    rl::sg::so::Model* sceneModel = static_cast< rl::sg::so::Model* > (scene->getModel(0));

    rl::kin::Kinematics* kinematics = rl::kin::Kinematics::create("../xml/rlkin/unimation-puma560.xml");
    kinematics->world() = ::rl::math::AngleAxis(
                90 * rl::math::DEG2RAD,
                ::rl::math::Vector3::UnitZ());

    kinematics->world().translation().x() = 0;
    kinematics->world().translation().y() = 0;
    kinematics->world().translation().z() = 0;

    this->model.reset(new rl::plan::Model());
    this->model->kin = kinematics;
    this->model->model = sceneModel;
    this->model->scene = scene.get();

    this->viewer->sceneGroup->addChild(scene->root);
    this->viewer->model = this->model.get();
    this->viewer->delta = 1.0 * rl::math::DEG2RAD;

    this->toggleView(true);
    this->toggleViewAction->setChecked(true);

    this->viewer->viewer->setBackgroundColor(SbColor(0.0f, 0.0f, 0.0f));
    this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
    this->viewer->viewer->getCamera()->setToDefaults();
    this->viewer->viewer->viewAll();
    this->viewer->viewer->getCamera()->position.setValue(
                (this->viewer->viewer->getCamera()->position.getValue()[0]),
                (this->viewer->viewer->getCamera()->position.getValue()[1]),
                (this->viewer->viewer->getCamera()->position.getValue()[2])
                );

    this->viewer->viewer->getCamera()->scaleHeight(1.0f);

    this->viewer->drawConfiguration(system->getStartConfiguration());
}

void
QtWindow::open()
{

}


void
QtWindow::reset()
{
    this->planningThread->stop();
    this->system->reset();
    this->model->reset();
    this->viewer->reset();

    this->system->setViewer(this->viewer);

    this->viewer->drawConfiguration(this->system->getConfiguration());

    this->configurationView->setEnabled(true);
    this->getGoalConfigurationAction-> setEnabled(true);
    this->getRandomConfigurationAction->setEnabled(true);
    this->getRandomFreeConfigurationAction->setEnabled(true);
    this->getStartConfigurationAction->setEnabled(true);
    this->plannerView->setEnabled(true);
    this->setGoalConfigurationAction->setEnabled(true);
    this->setStartConfigurationAction->setEnabled(true);
    this->startPlanningAction->setEnabled(true);
    this->toggleViewAction->setEnabled(true);
}

void
QtWindow::saveImage()
{
    this->viewer->saveImage("rlplan-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".png");
}

void
QtWindow::saveScene()
{
    this->viewer->saveScene("rlplan-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz") + ".wrl");
}

void
QtWindow::setGoalConfiguration()
{
    this->system->setGoalConfiguration(this->system->getConfiguration());
}

void
QtWindow::setStartConfiguration()
{
    this->system->setStartConfiguration(this->system->getConfiguration());
}

void
QtWindow::startPlanning()
{
    this->configurationView->setEnabled(false);
    this->getGoalConfigurationAction->setEnabled(false);
    this->getRandomConfigurationAction->setEnabled(false);
    this->getRandomFreeConfigurationAction->setEnabled(false);
    this->getStartConfigurationAction->setEnabled(false);
    this->plannerView->setEnabled(false);
    this->setGoalConfigurationAction->setEnabled(false);
    this->setStartConfigurationAction->setEnabled(false);
    this->startPlanningAction->setEnabled(false);
    this->toggleViewAction->setEnabled(false);

    this->system->setViewer(this->planningThread);

    this->planningThread->start();
}

void
QtWindow::toggleCamera()
{
    if (SoPerspectiveCamera::getClassTypeId() == this->viewer->viewer->getCameraType())
    {
        this->viewer->viewer->setCameraType(SoOrthographicCamera::getClassTypeId());
    }
    else
    {
        this->viewer->viewer->setCameraType(SoPerspectiveCamera::getClassTypeId());
    }

    SbVec3f position = this->viewer->viewer->getCamera()->position.getValue();
    SbRotation orientation = this->viewer->viewer->getCamera()->orientation.getValue();
    this->viewer->viewer->getCamera()->setToDefaults();
    this->viewer->viewer->getCamera()->position.setValue(position);
    this->viewer->viewer->getCamera()->orientation.setValue(orientation);
    this->viewer->viewer->viewAll();
}

void
QtWindow::toggleConfiguration()
{
    if (this->configurationDockWidget->isVisible())
    {
        this->configurationDockWidget->hide();
    }
    else
    {
        this->configurationDockWidget->show();
    }
}

void
QtWindow::toggleView(const bool& doOn)
{
    if (doOn)
    {
        this->viewer->viewer->setAutoRedraw(true);
        this->connect(this->planningThread, this->viewer);
    }
    else
    {
        this->disconnect(this->planningThread, this->viewer);
        this->viewer->viewer->setAutoRedraw(false);
    }
}
