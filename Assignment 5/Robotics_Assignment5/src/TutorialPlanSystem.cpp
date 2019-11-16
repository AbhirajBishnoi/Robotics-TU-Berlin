#include <fstream>
#include <QDateTime>
#include "TutorialPlanSystem.h"
#include "rl/math/Unit.h"
#include "rl/math/Rotation.h"
#include "rl/plan/UniformSampler.h"
#include <iostream>


TutorialPlanSystem::TutorialPlanSystem()
{
  //  Loading the scene from an predefined xml file which contains the convex model of the robot as well as the sourroundings
  //  Here's the collision scene where the puma 560 is loaded.
  rl::sg::bullet::Scene* scene = new rl::sg::bullet::Scene();
  scene->load("../xml/rlsg/unimation-puma560-rbo_wall.xml");
  rl::sg::bullet::Model* sceneModel = static_cast< rl::sg::bullet::Model* > (scene->getModel(0));

  //  Loading the kinematics of the puma 560 from a predefined xml file
  rl::kin::Kinematics* kinematics = rl::kin::Kinematics::create("../xml/rlkin/unimation-puma560.xml");
  kinematics->world() = ::rl::math::AngleAxis(90 * rl::math::DEG2RAD, ::rl::math::Vector3::UnitZ());
  kinematics->world().translation().x() = 0;
  kinematics->world().translation().y() = 0;
  kinematics->world().translation().z() = 0;

  //  Adding the robot kinematics and the scene to our internal model
  this->model.kin = kinematics;
  this->model.model = sceneModel;
  this->model.scene = scene;

  //  Setting the start, goal and current position
  this->start.resize(kinematics->getDof());
  this->goal.resize(kinematics->getDof());
  this->q.resize(kinematics->getDof());

  //  Start:
  this->start <<  0,0,90 * rl::math::DEG2RAD,0,0,0;

  //  Goal:
  this->goal <<  -98 * rl::math::DEG2RAD,
      7  * rl::math::DEG2RAD,
      44 * rl::math::DEG2RAD,
      0  * rl::math::DEG2RAD,
      44 * rl::math::DEG2RAD,
      0  * rl::math::DEG2RAD;

  //  Current position is at the start
  this->q = this->start;

  //  Set the model of the sampler to the system model
  this->sampler.model = &this->model;

  //  --- Parametrize the planner ---
  //  Delta defines the configuration step width of a connect attempt.
  //  Here delta is set to 1° => 1° steps are checked during a connect.
  //  Please don't change this value, the robot might collide if delta is too large.
  this->planner.delta = 1 * rl::math::DEG2RAD;

  //  Epsilon defines the distance between two configurations at which they are
  //  just identified as being identical.
  this->planner.epsilon = 1.0e-8f;

  //  duration defines the time interval in which the planner tries to solve the problem.
  //  Here the planner stops after 120 seconds of finding no solution.
  this->planner.duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<float>(1200.0)); //[s]

  //  Setting the start and the goal position of the planner.
  this->planner.goal = &this->goal;
  this->planner.start = &this->start;

  //  Set the sampler and the model of the planner.
  this->planner.sampler = &this->sampler;
  this->planner.model = &this->model;

  //  Set the parameters of the optimizer - you do not need to change these
  this->optimizer.length = 15 * rl::math::DEG2RAD;
  this->optimizer.ratio = 0.05;
  this->verifier.delta = 1 * rl::math::DEG2RAD;
  this->verifier.model = &this->model;
  this->optimizer.verifier = &this->verifier;
  this->optimizer.model = &this->model;

}

TutorialPlanSystem::~TutorialPlanSystem()
{
  //Free used memory
  delete this->model.kin;
  delete this->model.model;
  delete this->model.scene;
}

void TutorialPlanSystem::getRandomConfiguration(rl::math::Vector & config)
{
  //  By calling generate the sampler returns a random configuration
  //  of the robot in our system model
  config = sampler.generate();
}

void TutorialPlanSystem::getRandomFreeConfiguration(rl::math::Vector & config)
{
  //  By calling generateCollisionFree the sampler calls generate until it finds a
  //  non-colliding configuration
  config = sampler.generateCollisionFree();
}

void TutorialPlanSystem::writeToFile(rl::plan::VectorList & path)
{
  std::ofstream traj;
  traj.open("trajectory.txt", std::ios::trunc);

  //Print forwards
  rl::plan::VectorList::iterator it=path.begin();
  for(it; it!=path.end(); it++)
  {
    ::rl::math::Vector p=*it;
    for(size_t j=0;j<this->model.kin->getDof();j++)
    {
      traj<<p[j]<<" ";
    }
    traj<<std::endl;
  }

  //And backwards
  it--;
  do
  {
    it--;
    ::rl::math::Vector p=*it;
    for(size_t j=0;j<this->model.kin->getDof();j++)
    {
      traj<<p[j]<<" ";
    }
    traj<<std::endl;
  }while(it!=path.begin());

  traj.close();
}

bool TutorialPlanSystem::plan(rl::plan::VectorList & path)
{
    bool solved = false;
    double avg_vertices = 0;
    double avg_time = 0;
    double std_deviation = 0;
    double avg_total_queries = 0;
    double avg_free_queries = 0;

    int number_of_loops = 1;

 for(int i = 0; i < number_of_loops; i++) {

     this->reset();
     //Verifies that the model, the start, and the goal position are all correct
     if (!this->planner.verify()) {
         std::cout << "start or goal invalid" << std::endl;
         return false;
     }

     //Call the planner to solve the current problem.
     std::cout << "solve() ... " << std::endl;;
     std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
     solved = this->planner.solve();
     std::chrono::steady_clock::time_point stop = std::chrono::steady_clock::now();

     double plannerDuration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start).count() * 1000;
     //this->showMessage("Planner " + std::string(solved ? "succeeded" : "failed") + " in " + QString::number(plannerDuration).toStdString() + " ms.");

     std::cout << "solve() " << (solved ? "true" : "false") << " " << QString::number(plannerDuration).toStdString()
               << " ms" << std::endl;


     //write statistics to file benchmark.csv
     //format: date, time, solved, Planner name, # vertices, # Collision queries, # non-colliding queries, running time
     std::ofstream benchmark;
     benchmark.open("benchmark.csv", std::ios::app);
     benchmark << QDateTime::currentDateTime().toString("yyyy-MM-dd,HH:mm:ss.zzz").toStdString();
     benchmark << ",";
     benchmark << (solved ? "true" : "false");
     benchmark << ",";
     benchmark << this->planner.getName();
     benchmark << ",";
     benchmark << this->planner.getNumVertices();
     benchmark << ",";
     benchmark << this->model.getTotalQueries();
     benchmark << ",";
     benchmark << this->model.getFreeQueries();
     benchmark << ",";
     benchmark << plannerDuration;
     benchmark << std::endl;


     avg_vertices += this->planner.getNumVertices();
     avg_time += plannerDuration;
     avg_free_queries += this->model.getFreeQueries();
     avg_total_queries += this->model.getTotalQueries();


     //Check if the planner could solve in time
     if (solved) {
         //Found a solution so return the found path
         path = this->planner.getPath();

         std::cout << "optimize() ... " << std::endl;;

         //optimize the trajectory.
         //Comment this line if you only want to test your planning algorithm
         this->optimizer.process(path);

         //Write trajectory to text file
         writeToFile(path);
     }
 }

  avg_vertices /= number_of_loops;
  avg_time /=number_of_loops;
  avg_free_queries /=number_of_loops;
  avg_total_queries /=number_of_loops;

  std::cout << "avg_vertices: " << QString::number(avg_vertices).toStdString() << " " << "avg_time:" << " " << QString::number(avg_time).toStdString()<< " " <<
  "avg_free_queries:" << " " << QString::number(avg_free_queries).toStdString() << " " <<
                                      "avg_total_queries:"<< " " <<QString::number(avg_total_queries).toStdString()
                                                            <<std::endl;


    return solved;
}

void TutorialPlanSystem::reset()
{
  //Reset the planner and the model
  this->planner.reset();
  this->model.reset();
}

