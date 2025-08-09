#include <CameraWrapper.hpp>
#include <stereocam.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>


using namespace std::chrono_literals;

// Results
char outfile[100];
bool calibrated;
cv::Mat right_camera_mat = cv::Mat::eye(3, 3, CV_32FC1);
cv::Mat left_camera_mat = cv::Mat::eye(3, 3, CV_32FC1);
cv::Mat  left_dist_coef, right_dist_coef, R, T, E, F;

const int N = 10;
const auto find_flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE;
const auto calib_flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | 
                    cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;

const int checker_width = 10;
const int checker_height = 7;
const float checker_size = 1.5;


auto make_object_points()
{
    std::vector<cv::Point3f> out;
    for(int h = 0; h < checker_height; ++h)
    {
        for(int w = 0; w < checker_width; ++w)
        {
            out.push_back({h*checker_size, w*checker_size, 0.0});
        }
    }

    return out;
}


void stop()
{   
    std::cout << "Ending calibration program" << std::endl;
    if(calibrated)
    {
        std::cout << "Writing calibration results" << std::endl;
        std::ofstream file;
        file.open(outfile);
        file << "Stereo camera calibration results" << std::endl;
        file << "Left Intrinsic (cvtype " << left_camera_mat.type() << ", size " << left_camera_mat.size() << "): " << left_camera_mat << std::endl;
        file << "Left Distortion (cvtype " << left_dist_coef.type() << ", size " << left_dist_coef.size() << "): " << left_dist_coef << std::endl;
        file << "Right Intrinsic (cvtype " << right_camera_mat.type() << ", size " << right_camera_mat.size() << "): " << right_camera_mat << std::endl;
        file << "Right Distortion (cvtype " << right_dist_coef.type() << ", size " << right_dist_coef.size() << "): " << right_dist_coef << std::endl;
        file << "R (cvtype " << R.type() << ", size " << R.size() << "): " << R << std::endl;
        file << "T (cvtype " << T.type() << ", size " << T.size() << "): " << T << std::endl;
        file << "E (cvtype " << E.type() << ", size " << E.size() << "): " << E << std::endl;
        file << "F (cvtype " << F.type() << ", size " << F.size() << "): " << F << std::endl;
        file.close();
    }
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
    if(argc < 2)
    {
        std::cerr << "Output file required" << std::endl;
        return -1;
    }
    strcpy(outfile, argv[1]);
    calibrated = false;

    stereo::setup();
    stereo::start();
    auto obj_points = make_object_points();
    std::this_thread::sleep_for(1s); // Makes sure image data is populated before calibrating
    
    bool left_checker_ok = false, right_checker_ok = false;

    std::vector<std::vector<cv::Point2f>> all_left_points, all_right_points;
    std::vector<decltype(obj_points)> all_obj_points;
    cv::Size size;
    
    cv::Mat left_im, right_im;

    int i = 0;
    while(i < N)
    {        
        std::cout << "Collecting calibration sample " << i+1 << " of " << N << std::endl;
        
        stereo::left->lock();
        stereo::right->lock();
        left_im = stereo::left->image()->clone();
        right_im = stereo::right->image()->clone();
        size = stereo::left->image()->size();
        stereo::left->unlock();
        stereo::right->unlock();
        
        std::vector<cv::Point2f> left_points, right_points;
        left_checker_ok = cv::findChessboardCorners(left_im, cv::Size(checker_width, checker_height), left_points, find_flags);
        right_checker_ok = cv::findChessboardCorners(right_im, cv::Size(checker_width, checker_height), right_points, find_flags);

        auto left_im_draw = left_im.clone();
        cv::drawChessboardCorners(left_im_draw, cv::Size(checker_width, checker_height), left_points, left_checker_ok);
        char fname[50];
        sprintf(fname, "checker-%d.png", i+1);
        cv::imwrite(fname, left_im_draw);

        if(!(left_checker_ok && right_checker_ok))
        {
            std::cout << "Failed to identify valid checkerboard pattern. Trying again." << std::endl;
        }
        else
        {
            std::cout << "Checkerboard identified, refining corner positions." << std::endl;
            cv::cornerSubPix(left_im, left_points, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001));
            cv::cornerSubPix(right_im, right_points, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 30, 0.001));
            std::cout << "Saving calibration pattern" << std::endl;
            all_right_points.push_back(right_points);
            all_left_points.push_back(left_points);
            all_obj_points.push_back(obj_points);
            
            i += 1;
        }
        std::this_thread::sleep_for(5s); 
    }

    std::vector<cv::Mat> rvecsl, tvecsl, rvecsr, tvecsr;
    std::cout << "Calculating stereo calibration results." << std::endl;
    double rms = cv::stereoCalibrate(all_obj_points, all_left_points, all_right_points,
                left_camera_mat, left_dist_coef, right_camera_mat, right_dist_coef, size,
                R, T, E, F,
                calib_flags
            );
    std::cout << "Calibration done, RMS " << std::round(rms * 1000.0) / 1000.0 << std::endl;
    calibrated = true;

    stop();
    return 0;
}