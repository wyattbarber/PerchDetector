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


static double roundToNearest(double num) {
  return (num > 0.0) ? floor(num + 0.5) : ceil(num - 0.5);
}

Hough::Hough(const Eigen::Vector<double, 3>& minP, const Eigen::Vector<double, 3>& maxP, double var_dx,
             unsigned int sphereGranularity) {

  // compute directional vectors
  sphere = new Sphere();
  sphere->fromIcosahedron(sphereGranularity);
  num_b = sphere->vertices.size();

  // compute x'y' discretization
  max_x = std::max(maxP.norm(), minP.norm());
  double range_x = 2 * max_x;
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
void Hough::add(const  Eigen::Matrix<double, 3, Eigen::Dynamic> &pc) {
  for (int i = 0; i < pc.cols(); ++i) {
    pointVote(pc.col(i), true);
  }
}

// subtract all points from point cloud to voting space
void Hough::subtract(const  Eigen::Matrix<double, 3, Eigen::Dynamic> &pc) {
  for (int i = 0; i < pc.cols(); ++i) {
    pointVote(pc.col(i), false);
  }
}

// add or subtract (add==false) one point from voting space
// (corresponds to inner loop of Algorithm 2 in IPOL paper)
void Hough::pointVote(const Eigen::Vector<double, 3>& point, bool add){

  int inc_dir = add ? 1 : -1;

  // loop over directions B
  for(size_t j = 0; j < sphere->vertices.size(); j++) {

    const Vector3d& b = sphere->vertices[j];
    double beta = 1 / (1 + b.z);	// denominator in Eq. (2)

    // compute x' according to left hand side of Eq. (2)
    double x_new = ((1 - (beta * (b.x * b.x))) * point(0))
      - ((beta * (b.x * b.y)) * point(1))
      - (b.x * point(2));

    // compute y' according to right hand side Eq. (2)
    double y_new = ((-beta * (b.x * b.y)) * point(0))
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

// returns the line with most votes (rc = number of votes)
unsigned int Hough::getLine(Eigen::Vector<double, 3>& a, Eigen::Vector<double, 3>& b){
  unsigned int votes = 0;
  unsigned int index = 0;

  for(unsigned int i = 0; i < VotingSpace.size(); i++){
    if (VotingSpace[i] > votes) {
      votes = VotingSpace[i];
      index = i;
    }
  }

  // retrieve x' coordinate from VotingSpace[num_x * num_x * num_b]
  double x = (int) (index / (num_x * num_b));
  index -= (int) x * num_x * num_b;
  x = x * dx - max_x;

  // retrieve y' coordinate from VotingSpace[num_x * num_x * num_b]
  double y = (int) index / num_b;
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