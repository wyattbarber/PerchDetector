#include "loader.hpp"
#include "json/json.h"
#include <iostream>


int load_camera_matrices(const std::string& filename, std::string& err, cv::Mat& dist, cv::Mat& intr)
{
    std::ifstream file(filename);
    if(!file)
    {
        err = std::string("unable to open " ) +  filename;
        return -1;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, file, &root, &errs)) {
        err = std::string("Failed to parse ") + filename + std::string(": ") + errs;
        return -1;
    }
    if(!root.isMember("distortion") || !root.isMember("intrinsic"))
    {
        err = filename + std::string(" is missing 'distortion' or 'intrinsic'");
        return -1;
    }

    auto dist_json = root["distortion"];
    if(!dist_json.isArray() || !(dist_json.size() == 5))
    {
        err = std::string("distortion array is not the correct format");
        return -1;
    }    
    for(int i = 0; i < 5; i++)
    {   
        if(!dist_json[i].isDouble())
        {
            err = std::string("distortion array element ") + std::to_string(i) + std::string(" is not a double");
            return -1;
        }
        dist.at<double>(0,i) = dist_json[i].asDouble();
    }

    auto intr_json = root["intrinsic"];
    if(!intr_json.isArray() || intr_json.size() != 9)
    {
        err = std::string("intrinsic array is not the correct format");
        return -1;
    }    
    for(int i = 0; i < 9; i++)
    {   
        if(!intr_json[i].isDouble())
        {
            err = std::string("intrinsic array element ") + std::to_string(i) + std::string(" is not a double");
            return -1;
        }
        intr.at<double>(i/3, i%3) = intr_json[i].asDouble();
    }

    return 0;
}


int load_stereo_matrices(const std::string& filename, std::string& err, cv::Mat& R, cv::Mat& T)
{
    std::ifstream file(filename);
    if(!file)
    {
        err = std::string("unable to open ") + std::string(filename);
        return -1;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, file, &root, &errs)) {
        err = std::string("Failed to parse ") + filename + std::string(": ") + errs;
        return -1;
    }
    if(!root.isMember("translation") || !root.isMember("rotation"))
    {
        err = filename + std::string(" is missing 'translation' or 'rotation'");
        return -1;
    }

    auto tran_json = root["translation"];
    if(!tran_json.isArray() || !(tran_json.size() == 3))
    {
        err = "translation array is not the correct format";
        return -1;
    }    
    for(int i = 0; i < 3; i++)
    {   
        if(!tran_json[i].isDouble())
        {
            err = std::string("translation array element ") + std::to_string(i) + std::string(" is not a double");
            return -1;
        }
        T.at<double>(i,0) = tran_json[i].asDouble();
    }

    auto rot_json = root["rotation"];
    if(!rot_json.isArray() || rot_json.size() != 9)
    {
        err = "rotation array is not the correct format";
        return -1;
    }    
    for(int i = 0; i < 9; i++)
    {   
        if(!rot_json[i].isDouble())
        {
            err = std::string("rotation array element ") + std::to_string(i) + std::string(" is not a double");
            return -1;
        }
        R.at<double>(i/3, i%3) = rot_json[i].asDouble();
    }

    return 0;
}