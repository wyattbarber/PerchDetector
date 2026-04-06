#pragma once


#include <Eigen/Dense>
#include <hough.h>


std::pair<double, double> orthogonal_LSQ(const PointCloud &pc, Vector3d* a, Vector3d* b, Vector3d* c);


using Point = std::tuple<double, double, double>;
using Line = std::tuple<Point, Point, Point>;


auto hough3d(const std::vector<Point> &points, 
    size_t min_vote, 
    size_t maxlines, 
    size_t granularity, 
    double exp_width, double width_tol, 
    double min_ratio,
    double max_angle);

    
auto line_inliers(const std::vector<Point> &points, const Point &a, const Point &b);