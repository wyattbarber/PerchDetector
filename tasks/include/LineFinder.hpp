#pragma once


#include "DataSource.hpp"
#include "PointCloud.hpp"
#include <hough.h>


typedef struct {
    bool valid;
    double anchor[3];
    double dir[3];
    PointCloud::update_ptr_const_type pointcloud;
} LineFinderUpdate_t;

FWD_DECL_DATA_SOURCE(LineFinder, LineFinderUpdate_t)

/** CLI helper to save detection status.
 * 
 * Will save a binary file containing the detected line data and all 
 * inputs that went into detecting it. 
 * 
 * The binary file data ordering, starting from byte 0 is:
 *  - Line anchor [3 * sizeof(double)]
 *  - Line direction [3 * sizeof(double)]
 *  - N, Number of points in point cloud [sizeof(uint32_t)]
 *  - Point cloud vectors, (X, Y, X) form [N * 3 * sizeof(float)]
 *  - W, Image width (px) [sizeof(uint32_t)]
 *  - H, Image height (px) [sizeof(uint32_t)]
 *  - Left grayscale image pixels [W * H * sizeof(uint8_t)]
 *  - Right grayscale image pixels [W * H * sizeof(uint8_t)]
 *  - Confidence map [W * H * sizeof(uint8_t)]
 *  - Disparity map [W * H * sizeof(int16_t)]
 * 
 *  
 * The command accepts the following arguments:
 * 
 * Required, Positional:
 * - file: Name of the file to create and save data to.
 * 
 * Optional, Named:
 * - --next-valid: If given, will wait to save until the next valid data update. Otherwise saves the latest update.
 * 
 * 
 */
void save_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

/** Report out detection results to command line.
 * 
    Reports basic detections results like perch detected / not detected, distance, 
    position, and orientation.

 * The command accepts the following arguments:
 *  
 * Optional, Named:
 * - --next-valid: If given, will wait to report until the next valid data update. Otherwise reports the latest update.
*/
void report_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);


class LineFinder : public DataSource<LineFinder>
{
public:
    /** Create new point cloud generator.
    
    @param name Name of the new task.
    @param cloud PointCloud task to get data from.
    @param cal_path Path to folder containing calibration and settings.
    */
    LineFinder(const char* name, std::shared_ptr<PointCloud> cloud, const std::string& cal_path):
        DataSource<LineFinder>(name, {cloud}),
        cloud(cloud),        
        settings(cal_path + "/detector.json"),
        hough(nullptr)
    {
        declare_cli_command("save", &save_line_detect);
        declare_cli_command("report", &report_line_detect);
    }

    ~LineFinder()
    {
        if(hough) delete hough;
    }

    void step();

    bool start_impl();

    void stop_impl();

    std::vector<size_t> dims(){ return {1}; }
  
protected:
    std::shared_ptr<PointCloud> cloud;
    PointCloud::update_ptr_const_type latest;
    const std::string settings;
    Eigen::Vector<double, 3> center;

    unsigned min_vote;
    size_t max_lines;
    size_t granularity; 
    double min_width, max_width;
    double min_ratio;
    double max_angle;
    Hough* hough;
};