#pragma once

#include <opencv2/core/mat.hpp>
#include <fstream>



/** Load camera calibration parameters from a file.

Expects the file to be a json file containing the following objects:

*    distortion: A list of floats containing 5 elements

*    intrinsics: A list of floats containing 9 elements of a matrix. 
        Expects the list to contain all elements of the first row, 
        followed by all elements of the second row, then the third.

@param filename Fame of the file to read from
@param dist 1x5 distortion coefficient matrix
@param intr 3x3 intrinsic coefficient matrix

@return Result code, 0 if OK.
*/
int load_camera_matrices(const std::string& filename, cv::Mat& dist, cv::Mat& intr);


/** Load stereo camera parameters from a file.

Expects the file to be a json file containing the following objects:

*    translation: A list of floats containing 3 elements

*    rotation: A list of floats containing 9 elements of a matrix. 
        Expects the list to contain all elements of the first row, 
        followed by all elements of the second row, then the third.

@param filename Fame of the file to read from
@param R 3x3 rotation matrix
@param T 3x1 translation matrix

@return Result code, 0 if OK.
*/
int load_stereo_matrices(const std::string& filename, cv::Mat& R, cv::Mat& T);