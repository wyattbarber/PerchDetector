//
// HoughPointCloud.cpp
//     Class for holding a set of 3D points
//
// Author:  Tilman Schramke, Christoph Dalitz
// Date:    2017-03-16
// License: see LICENSE-BSD2
//

#include "pointcloud.h"
#include <stdio.h>
#include <math.h>
#include <string>

#include <iostream>


// store points closer than dx to line (a, b) in Y
Eigen::Matrix<double, 3, Eigen::Dynamic> pointsCloseToLine(const Eigen::Matrix<double, 3, Eigen::Dynamic>& X, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b, double dx)
{
  auto idx = indicesCloseToLine(X, a, b, dx);
  auto n = idx.size();
  Eigen::Matrix<double, 3, Eigen::Dynamic> out(3, n);
  for(size_t i = 0; i < n; ++i)
  {
    out.col(i) = X.col(idx[i]);
  }
  return out;
}

// store points closer than dx to line (a, b) in Y
std::vector<size_t> indicesCloseToLine(const Eigen::Matrix<double, 3, Eigen::Dynamic> &X, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b, double dx)
{
  double min_d = 10000000, max_d = 0.0;
  std::vector<size_t> indices;
  for (int i = 0; i < X.cols(); i++)
  {
    // distance computation after IPOL paper Eq. (7)
    double t = b.dot(X.col(i) - a);
    Eigen::Vector<double, 3> d = (X.col(i) - (a + (t * b)));
    double dn = d.norm();
    min_d = dn < min_d ? dn : min_d;
    max_d = dn > max_d ? dn : max_d;
    if (dn <= dx)
    {
      indices.push_back(i);
    }
  }
  std::cout << "Points ranged from " << min_d << " to " << max_d << " units away from the line with dx " << dx;
  return indices;
}

// removes the points in Y from HoughPointCloud
// WARNING: only works when points in same order as in HoughPointCloud!
Eigen::Matrix<double, 3, Eigen::Dynamic> removePoints(const Eigen::Matrix<double, 3, Eigen::Dynamic> &X, const Eigen::Matrix<double, 3, Eigen::Dynamic> &Y)
{

  if (Y.cols() == 0)
    return X;
  std::vector<size_t> newindices;
  newindices.reserve(X.cols() - Y.cols());
  int i, j;

  // important assumption: points in Y appear in same order in points
  for (i = 0, j = 0; i < X.cols() && j < Y.cols(); i++)
  {
    if (X.col(i) == Y.col(j))
    {
      j++;
    }
    else
    {
      newindices.push_back(i);
    }
  }
  // copy over rest after end of Y
  for (; i < X.cols(); i++)
    newindices.push_back(i);

  // Copy new data
  Eigen::Matrix<double, 3, Eigen::Dynamic> out(3, newindices.size());
  for (unsigned i = 0; i < out.cols(); ++i)
  {
    out.col(i) = X.col(newindices[i]);
  }
  return out;
}
