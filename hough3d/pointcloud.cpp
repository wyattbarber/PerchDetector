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


// translate point cloud so that center = origin
// total shift applied to this point cloud is stored in this->shift
void HoughPointCloud::shiftToOrigin(){
  Vector3d p1, p2, newshift;
  this->getMinMax3D(&p1, &p2);
  newshift = (p1 + p2) / 2.0;
  for(size_t i=0; i < points.size(); i++){
    points[i] = points[i] - newshift;
  }
  shift = shift + newshift;
}

// mean value of all points (center of gravity)
Vector3d HoughPointCloud::meanValue() const {
  Vector3d ret;
  for(size_t i = 0; i < points.size(); i++){
    ret = ret + points[i];
  }
  if (points.size() > 0)
    return (ret / (double)points.size());
  else
    return ret;
}

// bounding box corners
void HoughPointCloud::getMinMax3D(Vector3d* min_pt, Vector3d* max_pt){
  if(points.size() > 0){
    *min_pt = points[0];
    *max_pt = points[0];

    for(std::vector<Vector3d>::iterator it = points.begin(); it != points.end(); it++){
      if(min_pt->x > it->x) min_pt->x = it->x;
      if(min_pt->y > it->y) min_pt->y = it->y;
      if(min_pt->z > it->z) min_pt->z = it->z;

      if(max_pt->x < (*it).x) max_pt->x = (*it).x;
      if(max_pt->y < (*it).y) max_pt->y = (*it).y;
      if(max_pt->z < (*it).z) max_pt->z = (*it).z;
    }
  } else {
    *min_pt = Vector3d(0,0,0);
    *max_pt = Vector3d(0,0,0);
  }
}

// reads point cloud data from the given file
// (see IPOL paper for file format)
int HoughPointCloud::readFromFile(const char* path){
  FILE* f = fopen(path, "r");
  if (!f) return 1;

  char line[1024];
  Vector3d point;
  int n;
  while (fgets(line, 1023, f)) {
    if (line[0] == '#') continue;
    n = sscanf(line, "%lf,%lf,%lf", &point.x, &point.y, &point.z);
    if (n != 3) {
      fclose(f);
      return 2;
    }
    points.push_back(point);
#ifdef WEBDEMO
    if (points.size() > 1E6) {
      fprintf(stderr, "Error: program was compiled in WEBDEMO mode, "
              "which only permits less than 10^6 points in point cloud\n");
      return 3;
  }
#endif
  }

  fclose(f);
  return 0;
}

// store points closer than dx to line (a, b) in Y
void HoughPointCloud::pointsCloseToLine(const Vector3d &a, const Vector3d &b, double dx, HoughPointCloud* Y) {

  Y->points.clear();
  for (size_t i=0; i < points.size(); i++) {
    // distance computation after IPOL paper Eq. (7)
    double t = (b * (points[i] - a));
    Vector3d d = (points[i] - (a + (t*b)));
    if (d.norm() <= dx) {
      Y->points.push_back(points[i]);
    }
  }
}

// store points closer than dx to line (a, b) in Y
void HoughPointCloud::pointsCloseToLine(const Vector3d &a, const Vector3d &b, 
  double dx, std::vector<size_t>& indices) {
  for (size_t i=0; i < points.size(); i++) {
    // distance computation after IPOL paper Eq. (7)
    double t = (b * (points[i] - a));
    Vector3d d = (points[i] - (a + (t*b)));
    if (d.norm() <= dx) {
      indices.push_back(i);
    }
  }
}

// removes the points in Y from HoughPointCloud
// WARNING: only works when points in same order as in HoughPointCloud!
void HoughPointCloud::removePoints(const HoughPointCloud &Y){

  if (Y.points.empty()) return;
  std::vector<Vector3d> newpoints;
  size_t i,j;

  // important assumption: points in Y appear in same order in points
  for (i = 0, j = 0; i < points.size() && j < Y.points.size(); i++){
    if (points[i] == Y.points[j]) {
      j++;
    } else {
      newpoints.push_back(points[i]);
    }
  }
  // copy over rest after end of Y
  for (; i < points.size(); i++)
    newpoints.push_back(points[i]);

  points = newpoints;
}


Eigen::Matrix<double, 3, Eigen::Dynamic> HoughPointCloud::mat()
{
  Eigen::Matrix<double, 3, Eigen::Dynamic> out(3, points.size());
  for(unsigned i = 0; i < points.size(); ++i)
  {
    out(0, i) = points[i].x;
    out(1, i) = points[i].y;
    out(2, i) = points[i].z;
  }
  return out;
}