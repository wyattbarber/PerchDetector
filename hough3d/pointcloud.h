//
// PointCloud.h
//     Class for holding a set of 3D points
//
// Author:  Tilman Schramke, Christoph Dalitz
// Date:    2017-03-16
// License: see LICENSE-BSD2
//

#ifndef HoughPointCloud_H_
#define HoughPointCloud_H_

#include "vector3d.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

// store points closer than dx to line (a, b) in Y
Eigen::Matrix<double, 3, Eigen::Dynamic> pointsCloseToLine(const Eigen::Matrix<double, 3, Eigen::Dynamic>& X, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b,  double dx);
std::vector<size_t> indicesCloseToLine(const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y, const Eigen::Vector<double,3> &a, const Eigen::Vector<double,3> &b, double dx);
  
// removes the points in Y from HoughPointCloud
// WARNING: only works when points in same order as in HoughPointCloud!
std::vector<int> removePoints(const Eigen::Matrix<double, 3, Eigen::Dynamic>& X, const Eigen::Matrix<double, 3, Eigen::Dynamic>& Y);

#endif /* HoughPointCloud_H_ */
