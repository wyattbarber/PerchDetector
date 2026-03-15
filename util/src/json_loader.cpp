#include "json_loader.hpp"



bool load_camera_matrices(const std::string& filename, cv::Mat& dist, cv::Mat& intr)
{
    Json::Value root, dist_json, intr_json;
    if(!load_json_object(filename, root)) return false;
    if(!load_json_object(root, dist_json, "distortion"))
    {
        LOG_JSON_ERR("Failed to load distortion coefficients.");
        return false;
    }
    if(!load_json_object(root, intr_json, "intrinsic"))    
    {
        LOG_JSON_ERR("Failed to load intrinsic matrix.");
        return false;
    }

    if(!dist_json.isArray() || !(dist_json.size() == 5))
    {
        LOG_JSON_ERR("distortion array is not the correct format.");
        return false;
    }    
    for(int i = 0; i < 5; i++)
    {   
        if(!load_json_value(dist_json, dist.at<double>(0,i), i)) return false;
    }

    if(!intr_json.isArray() || intr_json.size() != 9)
    {
        LOG_JSON_ERR("intrinsic array is not the correct format");
        return false;
    }    
    for(int i = 0; i < 9; i++)
    {   
        if(!load_json_value(intr_json, intr.at<double>(i/3, i%3), i)) return false;
    }

    return true;
}


bool load_stereo_matrices(const std::string& filename, cv::Mat& R, cv::Mat& T)
{    
    Json::Value root, tran_json, rot_json;
    if(!load_json_object(filename, root)) return false;
    if(!load_json_object(root, tran_json, "translation"))
    {
        LOG_JSON_ERR("Failed to load translation vector.");
        return false;
    }
    if(!load_json_object(root, rot_json, "rotation"))    
    {
        LOG_JSON_ERR("Failed to load rotation matrix.");
        return false;
    }

    if(!tran_json.isArray() || !(tran_json.size() == 3))
    {
        LOG_JSON_ERR("translation array is not the correct format");
        return false;
    }    
    for(int i = 0; i < 3; i++)
    {   
        if(!load_json_value(tran_json, T.at<double>(i,0), i)) return false;
    }

    if(!rot_json.isArray() || rot_json.size() != 9)
    {
        LOG_JSON_ERR("rotation array is not the correct format");
        return false;
    }    
    for(int i = 0; i < 9; i++)
    {   
        if(!load_json_value(rot_json, R.at<double>(i/3, i%3), i)) return false;
    }

    return true;
}