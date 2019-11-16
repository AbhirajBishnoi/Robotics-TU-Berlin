#include "YourPlanner.h"
#include <rl/plan/Sampler.h>
#include <rl/plan/SimpleModel.h>
#include <rl/plan/Verifier.h>
#include <rl/plan/Viewer.h>
#include <boost/make_shared.hpp>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/point_cloud.h>

YourPlanner::YourPlanner() :
        RrtConConBase() {

    this->deltaDistance = 1.0;   //
    this->exhaustedThresh = 0;  //threshold for exhausted nodes
    this->clusterDistanceThresh = 3.0f; //
}

YourPlanner::~YourPlanner() {
}

::std::string
YourPlanner::getName() const {
    return "Awesome Planner";
}

void
YourPlanner::reset() {
    this->vertexClusterB.clear();
    this->vertexClusterA.clear();
    RrtConConBase::reset();
}

void
YourPlanner::choose(::rl::math::Vector &chosen) {
    //your modifications here
    RrtConConBase::choose(chosen);
}

RrtConConBase::Vertex
YourPlanner::connect(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen) {
    RrtConConBase::Vertex connected = RrtConConBase::connect(tree, nearest, chosen);


    // if connecting fails, exhaustionCount and radius of the vertex will be increased
    if (connected == NULL) {
        tree[nearest.first].exhaustCount += 1.0f;
        tree[nearest.first].radius = this->delta * 100;
  }

    return connected;
}

RrtConConBase::Vertex
YourPlanner::extend(Tree &tree, const Neighbor &nearest, const ::rl::math::Vector &chosen) {
    //your modifications here
    return RrtConConBase::extend(tree, nearest, chosen);
}

bool
YourPlanner::solve() {
//your modifications here
    this->time = ::std::chrono::steady_clock::now();
    // Define the roots of both trees
    this->begin[0] = this->addVertex(this->tree[0], ::std::make_shared<::rl::math::Vector>(*this->start));
    this->begin[1] = this->addVertex(this->tree[1], ::std::make_shared<::rl::math::Vector>(*this->goal));

    Tree *a = &this->tree[0];
    Tree *b = &this->tree[1];

    ::rl::math::Vector chosen(this->model->getDof());
    ::rl::math::Real minExhaustionVal = ::std::numeric_limits<::rl::math::Real>::max();
    ::rl::math::Real maxExhaustionVal = ::std::numeric_limits<::rl::math::Real>::min();

    while ((::std::chrono::steady_clock::now() - this->time) < this->duration)
  {
    //First grow tree a and then try to connect b.
    //then swap roles: first grow tree b and connect to a.
    for (::std::size_t j = 0; j < 2; ++j)
    {
      Neighbor aNearest;

      //change exhaustion Threshold base on the current maximum Exhaustionvalue of all nodes and take the highest 5%
      this->exhaustedThresh = 10+(maxExhaustionVal) * 0.95;
      while(true) {

        //Sample a random configuration
        this->choose(chosen);

        //Find the nearest neighbour in the tree
        aNearest = this->nearest(*a, chosen);

        // only take this neighbor if the sample is inside of the radius of the neighbor
        if(aNearest.second < (*a)[aNearest.first].radius){
          break;
        }
      }
      //Do a CONNECT step from the nearest neighbour to the sample
      Vertex aConnected = this->connect(*a, aNearest, chosen);

      // keep track of max_exhaustion value of the system as the node exhasution is updated after connection
      if (maxExhaustionVal < (*a)[aNearest.first].exhaustCount) {
          maxExhaustionVal = (*a)[aNearest.first].exhaustCount;
      }
            //If a new node was inserted tree a
            if (NULL != aConnected) {
                // Try a CONNECT step form the other tree to the sample
                Neighbor bNearest = this->nearest(*b, *(*a)[aConnected].q);
                Vertex bConnected = this->connect(*b, bNearest, *(*a)[aConnected].q);

                // keep track of max_exhaustion value of the system as the node exhasution is updated after connection aswell
                if (maxExhaustionVal < (*b)[bNearest.first].exhaustCount) {
                    maxExhaustionVal = (*b)[bNearest.first].exhaustCount;
                }
                if (NULL != bConnected) {
                    //Test if we could connect both trees with each other
                    if (this->areEqual(*(*a)[aConnected].q, *(*b)[bConnected].q)) {
                        this->end[0] = &this->tree[0] == a ? aConnected : bConnected;
                        this->end[1] = &this->tree[1] == b ? bConnected : aConnected;
                        return true;
                    }
                }
            }

            //Swap the roles of a and b
            using ::std::swap;
            swap(a, b);
        }

    }

    return false;
}


RrtConConBase::Neighbor
YourPlanner::nearest(const Tree &tree, const ::rl::math::Vector &chosen) {

    //create an empty pair <Vertex, distance> to return
    Neighbor p(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());

    //create an empty pair <Vertex, distance> as fallback, for the case that every vertex in that cluster is exhausted
    Neighbor p_fallback(Vertex(), (::std::numeric_limits<::rl::math::Real>::max)());


    //vector of vertices for storing nearest cluster
    std::vector<RrtConConBase::Vertex* > *nearestCluster;
    //variable to hold the current minimum distance value
    ::rl::math::Real minCentroidDistance = ::std::numeric_limits<::rl::math::Real>::max();


    //check which tree we're working with
    if (&tree == &this->tree[0]) {
        //iterate through clusters of tree A
        for (auto &cluster : this->vertexClusterA) {

            //calculate distance between chosen and cluster centroid
            ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*cluster.first].q);

            //check distance and update nearest cluster, if needed
            if (d < minCentroidDistance) {
                minCentroidDistance = d;
                nearestCluster = cluster.second;
            }
        }
    } else {
        //same if we're looking at tree B
        for (auto &cluster : this->vertexClusterB) {
            ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*cluster.first].q);

            if (d < minCentroidDistance) {
                minCentroidDistance = d;
                nearestCluster = cluster.second;
            }
        }
    }

    //iterate through all vertices of chosen nearest cluster
    for (auto &vertex : *nearestCluster) {

        //calculate distance between chosen and vertices of the nearest cluster
        ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*vertex].q);

        //check if distance is closer as current best
        if (d < p.second) {

            //check if vertex within exhaustion threshold
            if( tree[*vertex].exhaustCount <= this->exhaustedThresh) {
                p.first = *vertex;
                p.second = d;
            } else if (d < p_fallback.second) { // else check if better than current fallback
                p_fallback.first = *vertex;
                p_fallback.second = d;
            }
        }
    }

    //check if vertex within in threshold was not found, than use fallback
    if (p.second == ::std::numeric_limits<::rl::math::Real>::max()) {
        p.first = p_fallback.first;
        p.second = p_fallback.second;
    }


   /*
    //create an empty pair <Vertex, distance> to return
    //Neighbor p(Vertex(), (::std::numeric_limits< ::rl::math::Real >::max)());

    //Iterate through all vertices to find the nearest neighbour
    for (VertexIteratorPair i = ::boost::vertices(tree); i.first != i.second; ++i.first) {
        ::rl::math::Real d = this->model->transformedDistance(chosen, *tree[*i.first].q);

        if (d < p.second && tree[*i.first].exhaustCount < this->exhaustedThresh) {
            p.first = *i.first;
            p.second = d;
        }
    }
*/
    // Compute the square root of distance
    p.second = this->model->inverseOfTransformedDistance(p.second);

    return p;
}

RrtConConBase::Vertex
YourPlanner::addVertex(RrtConConBase::Tree &tree, const ::rl::plan::VectorPtr &q) {


    //use base addVertex functionality
    RrtConConBase::Vertex addedVertex = RrtConConBase::addVertex(tree, q);

    // variable for vertex cluster
    auto * vertexClusterVector = new std::vector<RrtConConBase::Vertex* >();

    // copy newly added vertex to have memory control
    auto * clusterVertex = new RrtConConBase::Vertex(addedVertex);


    //check current tree
    if (&tree == &this->tree[0]) {
        //check if there arent clusters yet - create new cluster then
        if (this->vertexClusterA.empty()) {
            vertexClusterVector->push_back(clusterVertex);
            this->vertexClusterA[clusterVertex] = vertexClusterVector;
        } else {
            // remember if, vertex was added to a cluster
            bool inserted = false;
            // iterate through all clusters
            for (auto &cluster : this->vertexClusterA) {
                // calculate distance to cluster centroid
                ::rl::math::Real distance = this->model->transformedDistance(*tree[*cluster.first].q, *q);
                //check if distance within threshold
                if (distance <= this->clusterDistanceThresh) {
                    //add vertex to the cluster and stop
                    vertexClusterVector = cluster.second;
                    vertexClusterVector->push_back(clusterVertex);
                    this->vertexClusterA[cluster.first] = vertexClusterVector;
                    inserted = true;
                    break;
                }
            }

            // if not matching cluster was found, create new cluster
            if (!inserted) {
                vertexClusterVector->push_back(clusterVertex);
                this->vertexClusterA[clusterVertex] = vertexClusterVector;
            }
        }
    } else {
        //same if tree B
        if (this->vertexClusterB.empty()) {
            vertexClusterVector->push_back(clusterVertex);
            this->vertexClusterB[clusterVertex] = vertexClusterVector;
        } else {
            bool inserted = false;
            for (auto &cluster : this->vertexClusterB) {
                ::rl::math::Real distance = this->model->transformedDistance(*tree[*cluster.first].q, *q);
                if (distance <= this->clusterDistanceThresh) {
                    vertexClusterVector = cluster.second;
                    vertexClusterVector->push_back(clusterVertex);
                    this->vertexClusterB[cluster.first] = vertexClusterVector;
                    inserted = true;
                    break;
                }
            }

            if (!inserted) {
                vertexClusterVector->push_back(clusterVertex);
                this->vertexClusterB[clusterVertex] = vertexClusterVector;
            }
        }
    }

    return addedVertex;
}
