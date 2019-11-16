#ifndef _TUTORIAL_PLAN_SYSTEM_H_
#define _TUTORIAL_PLAN_SYSTEM_H_

#include <rl/kin/Kinematics.h>
#include <rl/plan/DistanceModel.h>
#include <rl/plan/Optimizer.h>
#include <rl/plan/Planner.h>
#include <rl/plan/AdvancedOptimizer.h>
#include <rl/plan/RecursiveVerifier.h>
#include <rl/sg/so/Model.h>
#include <rl/sg/bullet/Model.h>
#include <rl/sg/so/Scene.h>
#include <rl/sg/bullet/Scene.h>

#include "YourPlanner.h"
#include "YourSampler.h"

class TutorialPlanSystem
{
public:
  TutorialPlanSystem();
  virtual ~TutorialPlanSystem();

  rl::math::Vector& getGoalConfiguration() {return goal;}
  void setGoalConfiguration(rl::math::Vector& config) {goal = config;}

  rl::math::Vector& getStartConfiguration() {return start;}
  void setStartConfiguration(rl::math::Vector& config) {start = config;}

  rl::math::Vector& getConfiguration() {return q;}
  void setConfiguration(rl::math::Vector& config) {q = config;}

  void getRandomConfiguration(rl::math::Vector & config);
  void getRandomFreeConfiguration(rl::math::Vector & config);

  void writeToFile(rl::plan::VectorList & path);

  void setViewer(rl::plan::Viewer* viewer) {this->planner.viewer = viewer;this->optimizer.viewer=viewer;}

  bool plan(rl::plan::VectorList &);

  void reset();

  rl::plan::DistanceModel& getModel() {return model;}

private:

  rl::math::Vector goal; //goal configuration
  rl::math::Vector start; //start configuration
  rl::math::Vector q; //current configuration

  rl::plan::DistanceModel model; //model for computation

  YourSampler sampler; //Sampler for random configurations

  rl::plan::AdvancedOptimizer optimizer; //Trajectory length optimizer
  rl::plan::RecursiveVerifier verifier; //The verifier for the optimizer

  YourPlanner planner;  //The implementation of your planner

};

#endif
