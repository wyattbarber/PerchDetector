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


class HoughPointCloud {
public:
  // translation of HoughPointCloud as done by shiftToOrigin()
  Vector3d shift;
  // points of the point cloud
  std::vector<Vector3d> points;

  // translate point cloud so that center = origin
  void shiftToOrigin();
  // mean value of all points (center of gravity)
  Vector3d meanValue() const;
  // bounding box corners
  void getMinMax3D(Vector3d* min_pt, Vector3d* max_pt);
  // reads point cloud data from the given file
  int readFromFile(const char* path);
  // store points closer than dx to line (a, b) in Y
  void pointsCloseToLine(const Vector3d &a, const Vector3d &b,
                         double dx, HoughPointCloud* Y);
  void pointsCloseToLine(const Vector3d &a, const Vector3d &b, double dx, std::vector<size_t>& indices);
  // removes the points in Y from HoughPointCloud
  // WARNING: only works when points in same order as in HoughPointCloud!
  void removePoints(const HoughPointCloud &Y);
  // Convert data to matrix
  Eigen::Matrix<double, 3, Eigen::Dynamic> mat();
};



#endif /* HoughPointCloud_H_ */
