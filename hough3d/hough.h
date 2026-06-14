//
// hough.h
//     Implementation of Algorithm 2 (Hough transform) from IPOL paper
//
// Author:  Tilman Schramke, Manuel Jeltsch, Christoph Dalitz
// Date:    2017-03-16
// License: see LICENSE-BSD2
//

#ifndef HOUGH_H_
#define HOUGH_H_

#include "sphere.h"
#include "pointcloud.h"
#include <vector>
#include <deque>
#include <Eigen/Dense>

#include <Eigen/Dense>


class Hough {
public:
  // accumulator array A
  std::vector<unsigned int> VotingSpace;
  // Directions B
  Sphere *sphere;
  size_t num_b;
  // x' and y'
  float dx, max_x;
  size_t num_x;

  // parameter space discretization and allocation of voting space
  Hough(const Eigen::Vector<float, 3>& minP, const Eigen::Vector<float, 3>& maxP, float dx,
        unsigned int sphereGranularity);
  ~Hough();
  // returns the line with most votes (rc = number of votes)
  unsigned int getLine(Eigen::Vector<float, 3>& point, Eigen::Vector<float, 3>& direction);
  // add all points from point cloud to voting space
  void add(const Eigen::Matrix<float, 3, Eigen::Dynamic> &pc);
  // subtract all points from point cloud to voting space
  void subtract(const Eigen::Matrix<float, 3, Eigen::Dynamic> &pc);
  // reset voting space so new point cloud can be accepted
  void reset();

private:
  // add or subtract (add==false) one point from voting space
  void pointVote(const Eigen::Vector<float, 3>& point, bool add);

};

#endif /* HOUGH_H_ */
