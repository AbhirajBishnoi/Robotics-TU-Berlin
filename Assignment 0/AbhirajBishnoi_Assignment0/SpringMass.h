// this is called an "include guard": it prevents
// the header from being inserted more than one time
// in the including h/cpp files. (Welcome to the 21st century!)
#ifndef SpringMass__H__
#define SpringMass__H__


#include <string>
#include <vector>
// DO NOT do this in a header file:
//  using namespace std;
// Reason: as #include only copies the content of a header file
//  into the cpp file, ALL cpp files including this header
//  would automatically have to use this namespace

/**
 * A two-dimensional array structure
 * 
 * By convention, x denotes the position of the object and y denotes the velocity.
 *
 * TODO
 */
// TODO define members, methods, constructors etc.
struct Vec2d {
  double x;
  double y;

  Vec2d(): x(0), y(0)
  {
  } 

};

/**
 * @brief SpringMass simulation class
 * 
 * Used to simulate the trajectory of a spring mass system,
 * spawned at t=0 at some initial position with some
 * initial velocity.
 */
class SpringMass {
  
public:

  std::vector<Vec2d> computed_states;
  int current_time;
  double pos_init;
  double vel_init;
  double pos_eqm;
  double vel_eqm;

  /**
   * @brief Construct the SpringMass object from initial position and velocity of the object,
   * position and velocity when the spring is unstretched (equilibrium state)
   */
  SpringMass(double pos_init, double vel_init, double pos_eqm, double vel_eqm);
  
  /**
   * @brief Destruct the SpringMass object
   */
  virtual ~SpringMass();
  
  /**
   * @brief Runs a step in the spring mass simulation
   * 
   * @return last time step t 
   */
  virtual int step();

  /**
   * @brief Get the position and velocity of the object at time t.
   * 
   * Constaints: t must be the current time step, or a time step
   * that has already passed. If these constraints are fullfilled,
   * the method returns true and stores position and velocity
   * in a Vec2d struct, otherwise the methods returns false.
   */
  virtual bool getConfiguration(int t, Vec2d& state) const;
  
  /**
   * @brief Return the current simulation time t
   */
  virtual int getCurrentSimulationTime() const;
  
  
protected:
  /**
   * @brief Given constants - use this for your simulation
   */
  static const double GRAVITY;
  static const double SPRING_CONST;
  static const double MASS;

  private:
 
};

#endif // SpringMass__H__
