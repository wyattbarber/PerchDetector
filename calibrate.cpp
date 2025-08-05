#include <CameraWrapper.hpp>
#include <stereocam.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;


const int N = 10;
const auto find_flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE;
const auto calib_flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | 
                    cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;

const int checker_width = 3;
const int checker_height = 4;
const float checker_size = 6.0;


std::vector<cv::Mat> left_camera_matrices, right_camera_matrices, left_dist_coeffs, right_dist_coeffs, Rs, Ts, Es, Fs;


auto make_object_points()
{
    std::vector<cv::Point3f> out;

    for(int w = 0; w < checker_width; ++w)
    {
        for(int h = 0; h < checker_height; ++h)
        {
            out.push_back({w*checker_size, h*checker_size, 0});
        }
    }

    return out;
}


void stop()
{   
    stereo::stop();
    stereo::teardown();
}


static void sig_handle(int signum)
{
    stop();
    exit(signum);
}


int main(int argc, char** argv)
{
    stereo::setup();
    stereo::start();
    auto obj_points = make_object_points();
    std::this_thread::sleep_for(1s); // Makes sure image data is populated before calibrating
    
    cv::Mat left_points = cv::Mat::zeros(checker_height*checker_width,2,CV_32FC1);
    cv::Mat right_points = cv::Mat::zeros(checker_height*checker_width,2,CV_32FC1);
    cv::Mat right_camera_mat = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat left_camera_mat = cv::Mat::zeros(3,3,CV_64F);
    cv::Mat left_dist_coef, right_dist_coef, R, T, E, F;
    bool left_checker_ok = false, right_checker_ok = false;

    int i = 0;
    while(i < N)
    {        
        std::cout << "Collecting calibration sample " << i+1 << " of " << N << std::endl;
        
        stereo::left->lock();
        stereo::right->lock();
        left_checker_ok = cv::findChessboardCorners(*stereo::left->image(), cv::Size(checker_width, checker_height), left_points, find_flags);
        right_checker_ok = cv::findChessboardCorners(*stereo::right->image(), cv::Size(checker_width, checker_height), right_points, find_flags);
        cv::Size size = stereo::left->image()->size();
        stereo::left->unlock();
        stereo::right->unlock();

        if(!(left_checker_ok && right_checker_ok))
        {
            std::cout << "Failed to identify checkerboard pattern. Trying again." << std::endl;
        }
        else
        {
            double rms = cv::stereoCalibrate(obj_points, left_points, right_points,
                left_camera_mat, left_dist_coef, right_camera_mat, right_dist_coef, size,
                R, T, E, F,
                calib_flags
            );
            left_camera_matrices.push_back(left_camera_mat);
            right_camera_matrices.push_back(right_camera_mat);
            left_dist_coeffs.push_back(left_dist_coef);
            right_dist_coeffs.push_back(right_dist_coef);
            Rs.push_back(R);
            Ts.push_back(T);
            Es.push_back(E);
            Fs.push_back(F);

            std::cout << "Collected calibration sample " << i+1 << " with RMS " << rms << std::endl;
        }

        std::this_thread::sleep_for(1s); 
    }

    stop();
    return 0;
}