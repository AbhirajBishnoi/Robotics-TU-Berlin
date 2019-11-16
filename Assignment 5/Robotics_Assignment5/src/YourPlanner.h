#ifndef _YOUR_PLANNER_H_
#define _YOUR_PLANNER_H_

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/point_cloud.h>
#include "RrtConConBase.h"

using namespace ::rl::plan;


/**
*	The implementation of your planner.
*	modify any of the existing methods to improve planning performance.
*/
class YourPlanner : public RrtConConBase
{
public:
  YourPlanner();

  virtual ~YourPlanner();

  virtual ::std::string getName() const;

  bool solve();

  void reset(); // Methoden-ÜBERHSCREIBEN NICHT VERGESSEN IN HEADER
protected:
  void choose(::rl::math::Vector& chosen);

  RrtConConBase::Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);

  RrtConConBase::Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  RrtConConBase::Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);


  private:
	double deltaDistance;
	double exhaustedThresh;
	//variable for threshold distance for assigning a vertex to a cluster
    ::rl::math::Real clusterDistanceThresh;

    //represents maps for tree A and B , which hold <centroid, vector of vertices>
    std::map<RrtConConBase::Vertex*, std::vector<RrtConConBase::Vertex* >* > vertexClusterA;
    std::map<RrtConConBase::Vertex*, std::vector<RrtConConBase::Vertex* >* > vertexClusterB;

//	pcl::KdTreeFLANN<::rl::math::Vector > kdTree;
};

#endif // _YOUR_PLANNER_H_
