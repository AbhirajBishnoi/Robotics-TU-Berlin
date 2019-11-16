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

#include <fstream>
#include <QApplication>
#include <QDateTime>
#include <QMutexLocker>
#include <rl/math/Quaternion.h>
#include <rl/math/Unit.h>
#include <rl/plan/Prm.h>
#include <rl/plan/Rrt.h>

#include "QtPlanningThread.h"

QtPlanningThread::QtPlanningThread(QMutex *mutex, TutorialPlanSystem* system, rl::plan::Viewer* viewer, QObject* parent) :
    QThread(parent),
    quit(false),
    swept(false),
    running(false)
{
    this->mutex = mutex;
    this->system = system;
    this->viewer = viewer;
}

QtPlanningThread::~QtPlanningThread()
{
}

void
QtPlanningThread::drawConfiguration(const rl::math::Vector& q)
{
    emit configurationRequested(q);
}

void
QtPlanningThread::drawConfigurationEdge(const rl::math::Vector& q0, const rl::math::Vector& q1, const bool& free)
{
    emit configurationEdgeRequested(q0, q1, free);
}

void
QtPlanningThread::drawConfigurationPath(const rl::plan::VectorList& path)
{
    emit configurationPathRequested(path);
}

void
QtPlanningThread::drawConfigurationVertex(const rl::math::Vector& q, const bool& free)
{
    emit configurationVertexRequested(q, free);
}

void
QtPlanningThread::drawLine(const rl::math::Vector& xyz0, const rl::math::Vector& xyz1)
{
    emit lineRequested(xyz0, xyz1);
}

void 
QtPlanningThread::drawPoint(const rl::math::Vector& xyz)
{
    emit pointRequested(xyz);	
}

void
QtPlanningThread::drawSphere(const rl::math::Vector& center, const rl::math::Real& radius)
{
    emit sphereRequested(center, radius);
}

void
QtPlanningThread::drawSweptVolume(const rl::plan::VectorList& path)
{
    emit sweptVolumeRequested(path);
}

void
QtPlanningThread::drawWork(const rl::math::Transform& t)
{
    emit workRequested(t);
}

void
QtPlanningThread::drawWorkEdge(const rl::math::Vector& q0, const rl::math::Vector& q1)
{
    //	emit workEdgeRequested(q0, q1);
}

void
QtPlanningThread::drawWorkPath(const rl::plan::VectorList& path)
{
    emit workPathRequested(path);
}

void
QtPlanningThread::drawWorkVertex(const rl::math::Vector& q)
{
    //	emit workVertexRequested(q);
}

void
QtPlanningThread::reset()
{
    emit resetRequested();
}

void
QtPlanningThread::resetEdges()
{
    emit edgeResetRequested();
}

void
QtPlanningThread::resetLines()
{
    emit lineResetRequested();
}

void
QtPlanningThread::resetPoints()
{
    emit pointResetRequested();
}

void
QtPlanningThread::resetSpheres()
{
    emit sphereResetRequested();
}


void
QtPlanningThread::resetVertices()
{
    emit vertexResetRequested();
}

void
QtPlanningThread::showMessage(const ::std::string& message)
{
    emit showMessageRequested(message);
}

void
QtPlanningThread::run()
{
    this->mutex->lock();
    this->running = true;
    this->mutex->unlock();

    this->drawConfiguration(system->getStartConfiguration());
    sleep(1);

    if(this->stopped())
    {
        return;
    }

    this->drawConfiguration(system->getGoalConfiguration());
    sleep(1);

    if(this->stopped())
    {
        return;
    }

    rl::plan::VectorList path;

    bool solved = this->system->plan(path);

    if(solved)
    {
        this->drawConfigurationPath(path);

        while (!this->stopped())
        {
            this->followPath(path);
        }
    }
    else
    {
        //this->sendFailure();
    }

}

void QtPlanningThread::followPath(rl::plan::VectorList& path)
{

	if(this->stopped())
    {
        return;
    }

    rl::plan::DistanceModel model = this->system->getModel();

    rl::math::Vector diff(model.getDof());
    rl::math::Vector inter(model.getDof());

    rl::plan::VectorList::iterator i = path.begin();
    rl::plan::VectorList::iterator j = ++path.begin();

    if (i != path.end() && j != path.end())
    {
        this->drawConfiguration(*i);
        msleep(10);
    }

    rl::math::Real delta = .01f;

    for (; i != path.end() && j != path.end(); ++i, ++j)
    {
        diff = *j - *i;

        rl::math::Real steps = std::ceil(model.distance(*i, *j) / delta);

        for (std::size_t k = 1; k < steps + 1; ++k)
        {
            if(this->stopped())
            {
                return;
            }

            model.interpolate(*i, *j, k / steps, inter);
            this->drawConfiguration(inter);
            msleep(10);
        }
    }

    if(this->stopped())
    {
        return;
    }

    rl::plan::VectorList::reverse_iterator ri = path.rbegin();
    rl::plan::VectorList::reverse_iterator rj = ++path.rbegin();

    if (ri != path.rend() && rj != path.rend())
    {
        this->drawConfiguration(*ri);
        msleep(10);
    }

    if(this->stopped())
    {
        return;
    }

    for (; ri != path.rend() && rj != path.rend(); ++ri, ++rj)
    {
        diff = *rj - *ri;

        rl::math::Real steps = std::ceil(model.distance(*ri, *rj) / delta);

        for (std::size_t k = 1; k < steps + 1; ++k)
        {
            if(this->stopped())
            {
                return;
            }

            model.interpolate(*ri, *rj, k / steps, inter);
            this->drawConfiguration(inter);
            msleep(10);
        }
    }
}

bool QtPlanningThread::stopped()
{
    this->mutex->lock();
    if (!this->running)
    {
        this->mutex->unlock();
        return true;
    }
    this->mutex->unlock();
    return false;
}

void
QtPlanningThread::stop()
{
    this->mutex->lock();
    if (this->running)
    {
        this->running = false;
        this->mutex->unlock();
        while (!this->isFinished())
        {
            QThread::usleep(0);
        }
    }
    else{
        this->mutex->unlock();
    }
}
