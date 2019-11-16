#include "SpringMass.h"
#include <ostream>
#include <fstream>
#include <iostream>

using namespace std;

// define gravity constant
const double SpringMass::GRAVITY = 10;
const double SpringMass::SPRING_CONST = 7;
const double SpringMass::MASS = 30;

SpringMass::SpringMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm)
{
    this->pos_init = pos_init;
    this->vel_init = vel_init;
    this->pos_eqm = pos_eqm;
    this->vel_eqm = vel_eqm;
    this->current_time = 0;
    Vec2d initial_state;
    initial_state.x = pos_init;
    initial_state.y = vel_init;
    computed_states.push_back(initial_state);
}

SpringMass::~SpringMass() {
    computed_states.clear();
}

int SpringMass::step() {
    current_time++;
    double new_vel = computed_states[current_time - 1].y -((SPRING_CONST / MASS) * (computed_states[current_time - 1].x - pos_eqm)); 
    double new_pos = computed_states[current_time - 1].x + new_vel;
    Vec2d new_state;
    new_state.x = new_pos; 
    new_state.y = new_vel;
    computed_states.push_back(new_state);
    return current_time; 
}

bool SpringMass::getConfiguration(int t, Vec2d& state) const {
  if(t > getCurrentSimulationTime())
      return false;
  else
  {
      state.x = computed_states[t].x;
      state.y = computed_states[t].y;
      return true;
  }
}

int SpringMass::getCurrentSimulationTime() const {
  return current_time; 
}
