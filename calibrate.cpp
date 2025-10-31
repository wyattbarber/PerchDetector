#include <CameraWrapper.hpp>
#include <stereocam.hpp>
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <thread>
#include <vector>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <Logging.hpp>


using namespace std::chrono_literals;
using namespace std;
using namespace cv;


// Results
char outfile[100];
bool calibrated;
cv::Mat right_camera_mat = cv::Mat::eye(3, 3, CV_32FC1);
cv::Mat left_camera_mat = cv::Mat::eye(3, 3, CV_32FC1);
cv::Mat  left_dist_coef, right_dist_coef, R, T, E, F;

// Settings
const int N = 10;
const auto find_flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE;
// const auto calib_flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH | 
//                     cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5;

// Create charuco board object and CharucoDetector
float squareLength = 3.40; // cm
float markerLength = 2.50; // cm
int squaresX = 6;
int squaresY = 4;
aruco::Dictionary dictLeft = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
aruco::Dictionary dictRight = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
aruco::CharucoBoard boardLeft(Size(squaresX, squaresY), squareLength, markerLength, dictLeft);
aruco::CharucoDetector detectorLeft(boardLeft);
aruco::CharucoBoard boardRight(Size(squaresX, squaresY), squareLength, markerLength, dictRight);
aruco::CharucoDetector detectorRight(boardRight);

// Collect data from each frame
vector<Mat> allCharucoCornersLeft, allCharucoIdsLeft, allCharucoCornersRight, allCharucoIdsRight;
vector<vector<Point2f>> allImagePointsLeft, allImagePointsRight;
vector<vector<Point3f>> allObjectPointsLeft, allObjectPointsRight;


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
    Logger::instance().set_file("calibration-log.txt");

    if(argc < 2)
    {
        Mat board_out;
        boardLeft.generateImage(Size(800, 600), board_out);
        imwrite("charuco-board.png", board_out);
        return 0;
    }
    strcpy(outfile, argv[1]);
    calibrated = false;

    allCharucoCornersLeft.reserve(N);
    allCharucoIdsLeft.reserve(N);
    allImagePointsLeft.reserve(N);
    allObjectPointsLeft.reserve(N);
    allCharucoCornersRight.reserve(N);
    allCharucoIdsRight.reserve(N);
    allImagePointsRight.reserve(N);
    allObjectPointsRight.reserve(N);

    stereo::setup();
    stereo::start();
    std::this_thread::sleep_for(1s); // Makes sure image data is populated before calibrating
        
    Mat left_im, right_im;
    Size size;

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

        imwrite("left.png", left_im);
        imwrite("right.png", right_im);
        
        vector<int> markerIdsLeft, markerIdsRight;
        vector<vector<Point2f>> markerCornersLeft, markerCornersRight;
        Mat currentCharucoCornersLeft, currentCharucoIdsLeft, currentCharucoCornersRight, currentCharucoIdsRight;
        vector<Point3f> currentObjectPointsLeft, currentObjectPointsRight;
        vector<Point2f> currentImagePointsLeft, currentImagePointsRight;
 
        // Detect ChArUco board
        detectorLeft.detectBoard(left_im, currentCharucoCornersLeft, currentCharucoIdsLeft);
        detectorRight.detectBoard(right_im, currentCharucoCornersRight, currentCharucoIdsRight);

        if(
            (currentCharucoCornersLeft.total() < boardLeft.getObjPoints().size())  || 
            (currentCharucoCornersRight.total() < boardRight.getObjPoints().size()) 
        )
        {
            std::cout << "Failed to identify valid checkerboard pattern. Trying again." << std::endl;
        }
        else
        {
            boardLeft.matchImagePoints( currentCharucoCornersLeft, 
                                        currentCharucoIdsLeft, 
                                        currentObjectPointsLeft, 
                                        currentImagePointsLeft
                                    );
            boardRight.matchImagePoints(currentCharucoCornersRight, 
                                        currentCharucoIdsRight, 
                                        currentObjectPointsRight, 
                                        currentImagePointsRight
                                    );
            if(currentImagePointsLeft.size() != currentObjectPointsLeft.size()) {
                cout << "Left image point matching failed, try again." << endl;
                continue;
            }
            if(currentImagePointsRight.size() != currentObjectPointsRight.size()) {
                cout << "Right image point matching failed, try again." << endl;
                continue;
            }            
            if(currentImagePointsLeft.size() != currentImagePointsRight.size())
            {
                cout << "Right and left point sets do not match" << endl;
                continue;
            }
 
            cout << "Frame captured" << endl;
 
            allCharucoCornersLeft.push_back(currentCharucoCornersLeft);
            allCharucoIdsLeft.push_back(currentCharucoIdsLeft);
            allImagePointsLeft.push_back(currentImagePointsLeft);
            allObjectPointsLeft.push_back(currentObjectPointsLeft);
            allCharucoCornersRight.push_back(currentCharucoCornersRight);
            allCharucoIdsRight.push_back(currentCharucoIdsRight);
            allImagePointsRight.push_back(currentImagePointsRight);
            allObjectPointsRight.push_back(currentObjectPointsRight);
 
            size = left_im.size();

            i += 1;
        }
        std::this_thread::sleep_for(5s); 
    }

    std::vector<cv::Mat> rvecsl, tvecsl, rvecsr, tvecsr;
    std::cout << "Calculating stereo calibration results." << std::endl;
    double rms = cv::stereoCalibrate(allObjectPointsLeft, allImagePointsLeft, allImagePointsRight,
                left_camera_mat, left_dist_coef, right_camera_mat, right_dist_coef, size,
                R, T, E, F
            );
    std::cout << "Calibration done, reprojection error: " << std::round(rms * 1000.0) / 1000.0 << std::endl;
    calibrated = true;

    stop();
    return 0;
}