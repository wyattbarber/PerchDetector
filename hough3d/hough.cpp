//
// hough.h
//     Implementation of Algorithm 2 (Hough transform) from IPOL paper
//
// Author:  Tilman Schramke, Manuel Jeltsch, Christoph Dalitz
// Date:    2017-03-16
// License: see LICENSE-BSD2
//

#include "hough.h"
#include <math.h>
#include "vector3d.h"
#include <cstdlib>


static float roundToNearest(float num) {
  return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}

Hough::Hough(const Eigen::Vector<float, 3>& minP, const Eigen::Vector<float, 3>& maxP, float var_dx,
             unsigned int sphereGranularity) {

  // compute directional vectors
  sphere = new Sphere();
  sphere->fromIcosahedron(sphereGranularity);
  num_b = sphere->vertices.size();

  // compute x'y' discretization
  max_x = std::max(maxP.norm(), minP.norm());
  float range_x = 2 * max_x;
  dx = var_dx;
  if (dx == 0.0) {
    // dx = range_x / 64.0;
    dx = range_x / 128.0;
  }
  num_x = roundToNearest(range_x / dx);

  // allocate voting space
  VotingSpace.resize(num_x * num_x * num_b);
  reset();
}

Hough::~Hough() {
  delete sphere;
}

// add all points from point cloud to voting space
void Hough::add(const  Eigen::Matrix<float, 3, Eigen::Dynamic> &pc) {
#ifdef HOUGH_VECTORIZE_VOTE
    vectorPointVote(pc, true);
#else
  for (int i = 0; i < pc.cols(); ++i) {
    pointVote(pc.col(i), true);
  }
#endif
}

// subtract all points from point cloud to voting space
void Hough::subtract(const  Eigen::Matrix<float, 3, Eigen::Dynamic> &pc) {
#ifdef HOUGH_VECTORIZE_VOTE
    vectorPointVote(pc, false);
#else
  for (int i = 0; i < pc.cols(); ++i) {
    pointVote(pc.col(i), false);
  }
#endif
}

// add or subtract (add==false) one point from voting space
// (corresponds to inner loop of Algorithm 2 in IPOL paper)
void Hough::pointVote(const Eigen::Vector<float, 3>& point, bool add){

  int inc_dir = add ? 1 : -1;

  // loop over directions B
  for(size_t j = 0; j < sphere->vertices.size(); j++) {

    const Vector3d& b = sphere->vertices[j];
    float beta = 1 / (1 + b.z);	// denominator in Eq. (2)

    // compute x' according to left hand side of Eq. (2)
    float x_new = ((1 - (beta * (b.x * b.x))) * point(0))
      - ((beta * (b.x * b.y)) * point(1))
      - (b.x * point(2));

    // compute y' according to right hand side Eq. (2)
    float y_new = ((-beta * (b.x * b.y)) * point(0))
      + ((1 - (beta * (b.y * b.y))) * point(1))
      - (b.y * point(2));

    size_t x_i = roundToNearest((x_new + max_x) / dx);
    size_t y_i = roundToNearest((y_new + max_x) / dx);

    // compute one-dimensional index from three indices
	// x_i * #planes * #direction_Vec + y_i * #direction_Vec + #loop
    size_t index = (x_i * num_x * num_b) + (y_i * num_b) + j;

    if (index < VotingSpace.size()) {
        VotingSpace[index] += inc_dir;
    }
  }
}


void Hough::vectorPointVote(const  Eigen::Matrix<float, 3, Eigen::Dynamic> &pc, bool add) {
  int inc_dir = add ? 1 : -1;
  Eigen::Matrix<int, 2, Eigen::Dynamic> linear_idx_helper = Eigen::Matrix<int, 2, Eigen::Dynamic>::Constant(2, pc.cols(), num_b);
  linear_idx_helper.row(0) *= num_x;

  // loop over directions B
  for(size_t j = 0; j < sphere->vertices.size(); j++) {
    const Vector3d& b = sphere->vertices[j];
    // u and v vectors that project each point into the x` y` plane for this direction vector
    float bzp1 = 1.0 + b.z;
    Eigen::Matrix<float, 2, 3> uv = {
      1.0 - (b.x*b.x / bzp1), -b.x*b.y / bzp1,    -b.x,
      -b.x*b.y / bzp1,        1.0 - (b.y / bzp1), -b.y  
    };
    // Project points into this direction vectors x`y` plane
    auto xyp = uv * pc;
    // Convert to discretized positions
    auto xyp_d = (xyp.array() + max_x) / dx;
    // Convert to linear indices
    Eigen::VectorXi indices = (xyp_d.array() * linear_idx_helper.array()).colwise().sum().array() + j;
    // Add votes to each index
    for(size_t i = 0; i < indices.size(), ++i) {
      if ((indices(i) >= 0) && (indices(i) < VotingSpace.size())) {
          VotingSpace[indices(i)] += inc_dir;
      }
    }
  }
}

// returns the line with most votes (rc = number of votes)
unsigned int Hough::getLine(Eigen::Vector<float, 3>& a, Eigen::Vector<float, 3>& b){
  unsigned int votes = 0;
  unsigned int index = 0;

  for(unsigned int i = 0; i < VotingSpace.size(); i++){
    if (VotingSpace[i] > votes) {
      votes = VotingSpace[i];
      index = i;
    }
  }

  // retrieve x' coordinate from VotingSpace[num_x * num_x * num_b]
  float x = (int) (index / (num_x * num_b));
  index -= (int) x * num_x * num_b;
  x = x * dx - max_x;

  // retrieve y' coordinate from VotingSpace[num_x * num_x * num_b]
  float y = (int) index / num_b;
  index -= (int) y * num_b;
  y = y * dx - max_x;

  // retrieve directional vector
  b = {sphere->vertices[index].x, sphere->vertices[index].y, sphere->vertices[index].z};

  // compute anchor point according to Eq. (3)
  a(0) = x * (1 - ((b(0) * b(0)) / (1 + b(2))))
    - y * ((b(0) * b(1)) / (1 + b(2)));
  a(1) = x * (-((b(0) * b(1)) / (1 + b(2))))
     + y * (1 - ((b(1) * b(1)) / (1 + b(2))));
  a(2) = - x * b(0) - y * b(1);

  return votes;
}


void Hough::reset(){
  for(auto& x: VotingSpace){
    x = 0;
  }
}