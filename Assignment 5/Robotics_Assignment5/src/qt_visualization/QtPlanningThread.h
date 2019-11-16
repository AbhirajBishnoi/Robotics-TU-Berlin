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

#ifndef _QT_PLANNING_THREAD_H_
#define _QT_PLANNING_THREAD_H_

#include <QThread>
#include <QMutex>
#include <rl/plan/Viewer.h>

#include "../TutorialPlanSystem.h"

class QtPlanningThread : public QThread, public rl::plan::Viewer
{
	Q_OBJECT
	
public:
        QtPlanningThread(QMutex *mutex, TutorialPlanSystem* system, rl::plan::Viewer* viewer, QObject* parent = NULL);
	
        virtual ~QtPlanningThread();
	
	void drawConfiguration(const rl::math::Vector& q);
	
	void drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free = true);
	
	void drawConfigurationPath(const rl::plan::VectorList& path);
	
	void drawConfigurationVertex(const rl::math::Vector& q, const bool& free = true);
	
	void drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void drawPoint(const rl::math::Vector& xyz);
	
	void drawSphere(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void drawSweptVolume(const rl::plan::VectorList& path);
	
	void drawWork(const rl::math::Transform& t);
	
	void drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void drawWorkPath(const rl::plan::VectorList& path);
	
	void drawWorkVertex(const rl::math::Vector& q);
	
	void reset();
	
	void resetEdges();
	
	void resetLines();
	
	void resetPoints();
	
	void resetSpheres();
	
	void resetVertices();

    void showMessage(const ::std::string& message);
	
	void run();
	
	void stop();
	
	bool quit;
	
	bool swept;
	
protected:
	
private:
        bool stopped();

        void followPath(rl::plan::VectorList& path);

	bool running;

        TutorialPlanSystem* system;

        rl::plan::Viewer* viewer;

        QMutex* mutex;
	
signals:
	void configurationRequested(const rl::math::Vector& q);
	
	void configurationEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free);
	
	void configurationVertexRequested(const rl::math::Vector& q, const bool& free);
	
	void configurationPathRequested(const rl::plan::VectorList& path);
	
	void edgeResetRequested();
	
	void lineRequested(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1);
	
	void lineResetRequested();
	
	void resetRequested();
	
	void sphereRequested(const rl::math::Vector& center, const rl::math::Real& radius);
	
	void sphereResetRequested();
	
	void pointRequested(const rl::math::Vector& xyz);
	
	void pointResetRequested();
	
	void sweptVolumeRequested(const rl::plan::VectorList& path);
	
	void vertexResetRequested();

    void showMessageRequested(const ::std::string& message);
	
	void workRequested(const rl::math::Transform& t);
	
	void workEdgeRequested(const rl::math::Vector& q0, const rl::math::Vector& q1);
	
	void workPathRequested(const rl::plan::VectorList& path);
	
	void workVertexRequested(const rl::math::Vector& q);
};

#endif // _QT_PLANNING_THREAD_H_
