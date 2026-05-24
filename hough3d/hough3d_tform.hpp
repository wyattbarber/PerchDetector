#pragma once


#include <Eigen/Dense>
#include <hough.h>



/** Line parameters

Tuple of 3 Eigen vectors, each with 3 double elements. The vectors are:

* anchor: The center of the line

* direction: Vector pointing along the major axis of the line, with length equal to half the measured line length.

* normal: Vector pointing along the minor axis of the line, with length equal to half the measured line width.

*/
using Line = std::tuple<Eigen::Vector<double, 3>,Eigen::Vector<double, 3>, Eigen::Vector<double, 3>>;


void orthogonal_LSQ(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, Eigen::Vector<double, 3>& a, Eigen::Vector<double, 3>& b, Eigen::Vector<double, 3>& c);

/** Perform a 3D Hough transform on a set of points.

@param points Point cloud to process.
@param hough Initialized hough transform evaluator.
@param min_vote Minimum votes required to count a line.
@param maxlines Maximum number of lines to count before exiting.
@param min_width Minimum line width to count as a detection.
@param max_width Maximum line width to count as a detection.
@param min_ratio Minimum ratio between magnitude of first and second principle components.
@param max_angle Maximum angle to allow between a detected line and the camera plane.

@return Vector of line parameters
*/
std::vector<Line> hough3d(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points,
    Hough& hough,
    size_t min_vote, 
    size_t maxlines, 
    double min_width, 
    double max_width, 
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


/** Measures the width of a point cloud along a direction.
 * 
 * 
 * 
 * @param points Point cloud to measure
 * @param dir Unit vector giving the direction along which to measure length.
 */
double projected_length(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points, const Eigen::Vector<double, 3>& dir);