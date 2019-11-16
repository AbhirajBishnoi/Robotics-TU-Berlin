/*
 
Declaration of class ForwardKinematicsPuma() and HTransform type.


*/




/*
This type holds a homogenous transform matrix 

First index is row, second index is column
*/
typedef float HTransform[4][4];



class ForwardKinematicsPuma2D {

public:
  float angles[3];  //Contains the joint angles along the kinematic chain, in degrees
  static const float l1; //length of link 1
  static const float l2; //length of link 2
  static const float l3; //length of link 3
  HTransform T0_1;  //Homogenous transform from Link 0 to Link 1, updated by computeT0_1()
  HTransform T1_2;  //Homogenous transform from Link 1 to Link 2, updated by computeT1_2()
  HTransform T2_3;  //Homogenous transform from Link 2 to Link 3, updated by computeT2_3()
  HTransform T3_E;  //Homogenous transform from Link 3 to end effector frame, updated by computeT3_E()
  HTransform T0_E;  //Homogenous transform from Link 0 to end effector frame, updated by computeT0_E()
  float F[3];       //operational space position of the end effector (x, y, alpha), updated by computeF()
  float J[3][3];    //end effector Jacobian, updated by computeJ()


    /*

    updates the variable T0_1
    */
    virtual void computeT0_1();
    /*
    updates the variable T1_2
    */
    virtual void computeT1_2();
    /*
    updates the variable T2_3
    */
    virtual void computeT2_3();
    /*
    updates the variable T3_E
    */
    virtual void computeT3_E();
    /*
    updates the variable T0_E
    */
    virtual void computeT0_E();
    /*
    */
    virtual void computeF();
    /*
    */
    virtual void computeJ();

    /*
    This function changes the joint angles and recomputes the dependent variables
     
    a1, a2 and a3 are assumed to be given in radians!
    */
    virtual void setJoints(float a1, float a2, float a3);
};

//(corrected) link lengths
const float ForwardKinematicsPuma2D::l1 = 0.4318 - 0.0203; //length of link 1
const float ForwardKinematicsPuma2D::l2 = 0.4331; //length of link 2
const float ForwardKinematicsPuma2D::l3 = 0.055; //length of link 3



