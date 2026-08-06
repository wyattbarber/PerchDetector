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
Eigen::Matrix<float, 3, Eigen::Dynamic> pointsCloseToLine(const Eigen::Matrix<float, 3, Eigen::Dynamic>& X, const Eigen::Vector<float, 3> &a, const Eigen::Vector<float, 3> &b,  float dx);
std::vector<Eigen::Index> indicesCloseToLine(const Eigen::Matrix<float, 3, Eigen::Dynamic>& Y, const Eigen::Vector<float,3> &a, const Eigen::Vector<float,3> &b, float dx);
std::vector<Eigen::Index> indicesCloseToLineVectorized(const Eigen::Matrix<float, 3, Eigen::Dynamic>& Y, const Eigen::Vector<float,3> &a, const Eigen::Vector<float,3> &b, float dx);
  

// removes the points in Y from HoughPointCloud
// WARNING: only works when points in same order as in HoughPointCloud!
std::vector<Eigen::Index> removePoints(const Eigen::Matrix<float, 3, Eigen::Dynamic>& X, const std::vector<Eigen::Index>& Y);

#endif /* HoughPointCloud_H_ */
