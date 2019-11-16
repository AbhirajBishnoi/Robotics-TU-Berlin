#define CATCH_CONFIG_MAIN  
#include "catch.h"

#include "SpringMass.h"
#include "SpringDamperMass.h"

#include <iostream>
#include <assert.h>
#include <math.h>

using namespace std;

TEST_CASE( "Spring mass output is checked", "[spring]" ) {
  SpringMass mass(200., 0, 161., 0);
  int t;
  Vec2d s;

  do
 {
    t = mass.step();
 } while (t < 2000 && t > 0);

  mass.getConfiguration(t, s);

  double expected[] = {141, -19};
  REQUIRE( mass.getCurrentSimulationTime() == 2000);
  REQUIRE( floor(s.x) == expected[0] );
  REQUIRE( floor(s.y) == expected[1] );


}

TEST_CASE( "Spring mass damper output is checked", "[spring_damper]" ) {
	
  SpringDamperMass mass(200., 0., 161., 0., 1.);
  int t;
  Vec2d s1, s2;

  do
 {
    t = mass.step();
 } while (t < 2000 && t > 0);

  mass.getConfiguration(10, s1);
  mass.getConfiguration(t, s2);
  
  double expected1[] = {174, 16};
  double expected2[] = {161, 0};

  
  REQUIRE( floor(s1.x) == expected1[0] );
  REQUIRE( floor(s1.y) == expected1[1] );
  REQUIRE( round(s2.x) == expected2[0] );
  REQUIRE( round(s2.y) == expected2[1] );

}

/*
TEST_CASE( "Spring mass output is printed to create a plot", "[spring]" ) {
  SpringMass mass(200., 0, 161., 0);
  int t;
  Vec2d s;

  do
 {
    t = mass.step();
    mass.getConfiguration(t,s);
    cout << t << "\t" << s.x << "\t" << s.y << "\n"; 
 } while (t < 500 && t > 0);

}

TEST_CASE( "Spring damped mass output is printed to create a plot", "[spring]" ) {
  SpringDamperMass mass(200., 0, 161., 0, 1.0);
  int t;
  Vec2d s;

  do
 {
    t = mass.step();
    mass.getConfiguration(t, s);
    cout << t << "\t" << s.x << "\t" << s.y << "\n"; 
 } while (t < 500 && t > 0);

}

*/

