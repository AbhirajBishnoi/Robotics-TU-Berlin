//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Modified and commented by Arne Sieverling
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef RRT_CON_CON_BASE_H
#define RRT_CON_CON_BASE_H

#include <boost/graph/adjacency_list.hpp>

#include <rl/plan/MatrixPtr.h>
#include <rl/plan/Model.h>
#include <rl/plan/Planner.h>
#include <rl/plan/Sampler.h>
#include <rl/plan/TransformPtr.h>
#include <rl/plan/VectorPtr.h>
#include <rl/plan/Verifier.h>
#include <limits>

/**
 * Rapidly-Exploring Random Trees.
 *
 * Steven M. LaValle. Rapidly-exploring random trees: A new tool for path
 * planning. Technical Report TR 98-11, Iowa State University, Ames, IA,
 * USA, October 1998.
 *
 * http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RrtConConBase : public rl::plan::Planner
{
public:
  RrtConConBase();

  virtual ~RrtConConBase();

  virtual ::std::string getName() const;

  virtual ::std::size_t getNumEdges() const;

  virtual ::std::size_t getNumVertices() const;

  virtual rl::plan::VectorList getPath();

  virtual void reset();

  virtual bool solve();

  /////////////////////////////////////////////////////////////////////////
  // Planner parameters ///////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////

  /** Configuration step size. */
  ::rl::math::Real delta;

  /** Epsilon for configuration comparison. */
  ::rl::math::Real epsilon;

  /** The sampler used for planning */
  ::rl::plan::Sampler* sampler;

protected:
  /////////////////////////////////////////////////////////////////////////
  // boost graph definitions //////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////

  /** This struct defines all variables that are stored in each tree vertex.
  You can access them by calling i.e.: tree[vertex].tmp
  If you need additional parameters for vertices add them here */
  struct VertexBundle
  {
    ::std::size_t index;

    ::rl::plan::VectorPtr q;

    ::rl::math::Real tmp;

    // attribute to count number of times the vertex was unable to connect

    float exhaustCount = 0.0f;

    double radius = std::numeric_limits<double>::max();
  };

  typedef ::boost::adjacency_list_traits<
  ::boost::listS,
  ::boost::listS,
  ::boost::bidirectionalS,
  ::boost::listS
  >::vertex_descriptor Vertex;

  /** This defines a boost graph */
  typedef ::boost::adjacency_list<
  ::boost::listS,
  ::boost::listS,
  ::boost::bidirectionalS,
  VertexBundle,
  ::boost::no_property,
  ::boost::no_property
  > Tree;

  typedef ::boost::graph_traits< Tree >::edge_descriptor Edge;

  typedef ::boost::graph_traits< Tree >::edge_iterator EdgeIterator;

  typedef ::std::pair< EdgeIterator, EdgeIterator > EdgeIteratorPair;

  typedef ::boost::graph_traits< Tree >::vertex_iterator VertexIterator;

  typedef ::std::pair< VertexIterator, VertexIterator > VertexIteratorPair;

  typedef ::std::pair< Vertex, ::rl::math::Real > Neighbor;

  ////////////////////////////////////////////////////////////////////////
  // helper functions ////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** Add an edge to the RR-Tree */
  virtual Edge addEdge(const Vertex& u, const Vertex& v, Tree& tree);

    virtual /** Add a vertex to the RR-Tree */
  Vertex addVertex(Tree& tree, const ::rl::plan::VectorPtr& q);

  bool areEqual(const ::rl::math::Vector& lhs, const ::rl::math::Vector& rhs) const;

  ////////////////////////////////////////////////////////////////////////
  // RRT functions ///////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** Draws a random sample configuration*/
  virtual void choose(::rl::math::Vector& chosen);

  /** Extends vertex nearest of tree towards sample chosen*/
  virtual Vertex extend(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  /** Tries to connect vertex nearest of tree to sample chosen*/
  virtual Vertex connect(Tree& tree, const Neighbor& nearest, const ::rl::math::Vector& chosen);

  /** Returns the nearest neighbour of chosen in tree*/
  virtual Neighbor nearest(const Tree& tree, const ::rl::math::Vector& chosen);

  ////////////////////////////////////////////////////////////////////////
  // members /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////

  /** A vector of RRTs - here it's size 2 because we use two trees that grow towards each other */
  ::std::vector< Tree > tree;

  /** Start and end of the solution path */
  ::std::vector< Vertex > begin;
  ::std::vector< Vertex > end;

private:

};


#endif // RL_PLAN_RRT_CON_CON_BASE_H
