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


// store points closer than dx to line (a, b) in Y
Eigen::Matrix<float, 3, Eigen::Dynamic> pointsCloseToLine(const Eigen::Matrix<float, 3, Eigen::Dynamic>& X, const Eigen::Vector<float, 3> &a, const Eigen::Vector<float, 3> &b, float dx)
{
  auto idx = indicesCloseToLine(X, a, b, dx);
  auto n = idx.size();
  Eigen::Matrix<float, 3, Eigen::Dynamic> out(3, n);
  for(size_t i = 0; i < n; ++i)
  {
    out.col(i) = X.col(idx[i]);
  }
  return out;
}

// store points closer than dx to line (a, b) in Y
std::vector<Eigen::Index> indicesCloseToLine(const Eigen::Matrix<float, 3, Eigen::Dynamic> &X, const Eigen::Vector<float, 3> &a, const Eigen::Vector<float, 3> &b, float dx)
{
#ifdef HOUGH_VECTORIZE_DIST
  return indicesCloseToLineVectorized(X, a, b, dx);
#else
  std::vector<Eigen::Index> indices;
  for (int i = 0; i < X.cols(); i++)
  {
    // distance computation after IPOL paper Eq. (7)
    float t = b.dot(X.col(i) - a);
    Eigen::Vector<float, 3> d = (X.col(i) - (a + (t * b)));
    if (d.norm() <= dx)
    {
      indices.push_back(i);
    }
  }
  return indices;
#endif
}

std::vector<Eigen::Index> indicesCloseToLineVectorized(const Eigen::Matrix<float, 3, Eigen::Dynamic> &X, const Eigen::Vector<float, 3> &a, const Eigen::Vector<float, 3> &b, float dx)
{
  std::vector<Eigen::Index> indices;
  auto t = b.transpose() * (X.colwise() - a);
  auto d = X.colwise() - (a + (b * t));
  Eigen::VectorXf n = d.colwise().norm();

  float* data = n.data();
  for (int i = 0; i < n.size(); i++)
  {
    if (*(data+i) <= dx)
    {
      indices.push_back(i);
    }
  }
  return indices;
}

// removes the points in Y from HoughPointCloud
// WARNING: only works when points in same order as in HoughPointCloud!
std::vector<Eigen::Index> removePoints(Eigen::Index N, const std::vector<Eigen::Index>& Y)
{
  std::vector<Eigen::Index> newindices;

  if (Y.size() == 0)
  {
    for(int i = 0; i < N; ++i)
    {
      newindices.push_back(i);
    }
    return newindices;
  }
    
  newindices.reserve(N - Y.size());
  int i, j;

  // important assumption: points in Y appear in same order in points
  for (i = 0, j = 0; i < N && j < Y.size(); i++)
  {
    if (i == Y.at(j))
    {
      j++;
    }
    else
    {
      newindices.push_back(i);
    }
  }
  // copy over rest after end of Y
  for (; i < N; i++)
    newindices.push_back(i);

  return newindices;
}
