#pragma once


#include <Eigen/Dense>
#include <hough.h>


std::pair<double, double> orthogonal_LSQ(const PointCloud &pc, Vector3d* a, Vector3d* b, Vector3d* c);


/** Perform a 3D Hough transform on a set of points.

@param min_vote Minimum votes required to count a line.
@param maxlines Maximum number of lines to count before exiting.
@param granularity Granularity of the parameter space.
@param max_width Maximum line width to count as a detection.
@param min_ratio Minimum ratio between magnitude of first and second principle components.
@param max_angle Maximum angle to allow between a detected line and the camera plane.

@return Vector of line parameters
*/
auto hough3d(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points, 
    size_t min_vote, 
    size_t maxlines, 
    size_t granularity, 
    double exp_width, double width_tol, 
    double min_ratio,
    double max_angle);


/** Get the inlier points for a specified line.

Returns the indices of the points in the point cloud that lie on the line
specified by the given anchor and direction.

@param a Anchor point of the line
@param b Unit vector pointing the direction of the line from the anchor

@return Vector of inlier indices
*/
auto line_inliers(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points, const Eigen::Vector<double, 3>& a, const Eigen::Vector<double, 3> &b);