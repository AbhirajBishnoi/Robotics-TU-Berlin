#ifndef SPRING_DAMPER_MASS__H__
#define SPRING_DAMPER_MASS__H__

#include "SpringMass.h"

class SpringDamperMass : public SpringMass {
public: 

  /**
   * @brief Initialize object initial position and velocity,  
   * object position and velocity when the spring is unstretched (equilibrium state)
   * and the damping coefficient
   */
 
  SpringDamperMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm,
           double _damping_coeff) : SpringMass(pos_init, vel_init, pos_eqm, vel_eqm)
  {
      damping_coeff = _damping_coeff;
  }

  int step(); 

private:
  /**
   * Damping coefficient for damper
   */ 
  double damping_coeff;

};


#endif 
