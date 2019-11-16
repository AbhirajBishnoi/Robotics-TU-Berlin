#include "SpringDamperMass.h"

int SpringDamperMass::step() {
    current_time++; 
    double new_vel = computed_states[current_time - 1].y -((damping_coeff / MASS) * (computed_states[current_time - 1].y)) -((SPRING_CONST / MASS) * (computed_states[current_time - 1].x - this->pos_eqm));
    double new_pos = computed_states[current_time - 1].x + new_vel;
    Vec2d new_state;
    new_state.x = new_pos;
    new_state.y = new_vel;
    computed_states.push_back(new_state);
    return current_time;
}

