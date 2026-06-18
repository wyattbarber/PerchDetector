#include "LineFinder.hpp"
#include <hough.h>
#include "json_loader.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>


/** Selects the best candidate line.

This function identifies the line that is most likely to be 
a suitable perch. The best method for this found so far is to simply
take the closest (smallest Z coordinate) line.

@param lines Vector of parameters for lines detected in a point cloud.

@return Index of the best candidate in the input vector.
*/
size_t best_line_idx(const std::vector<Line>& lines)
{
    float closest_z = std::numeric_limits<float>::infinity();
    size_t idx_best = 0;
    for(size_t i = 0; i < lines.size(); ++i)
    {
        if(lines[i].anchor[2] < closest_z)
        {
            closest_z = lines[i].anchor[2];
            idx_best = i;
        }
    }
    return idx_best;
}


void LineFinder::step()
{
    if(latest){
        if(!latest->stale) return; // No update
    }
    latest = cloud->acquire();
    tick();

    auto next = allocate_next();

    // Process new point cloud for lines
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> points(
        const_cast<float*>(latest->data.cloud),
        3, latest->data.n_valid
    );
    
    Eigen::Map<Eigen::Vector<int8_t, Eigen::Dynamic>> ids((int8_t*)&next->data.line_ids, latest->data.n_valid);
    ids.fill(-1);

    const auto lines = hough3d(points, ids);

    if(lines.size() > 0)
    {
        // Select the canditate line and form data update
        next->data.valid = true;
        const auto idx = best_line_idx(lines);
        next->data.n_lines = lines.size();
        next->data.selected_line = idx;

        for(size_t i = 0; i < std::min(static_cast<unsigned>(lines.size()), MAX_LINES); ++i)
        {
            memcpy((void*)next->data.lines[i].anchor, (void*)lines[i].anchor, 3*sizeof(float));
            memcpy((void*)next->data.lines[i].dir, (void*)lines[i].dir, 3*sizeof(float));
        }
        next->data.pointcloud = latest;
    }
    else
    {        
        // Flag update as invalid
        next->data.valid = false;
        next->data.pointcloud = latest;
        warning("No candidates detected.");
    }
    swap_data();
}


bool LineFinder::start_impl()
{
    double max_angle_deg;
    if(!load_json_value_pairs(settings,
        std::make_tuple(),
        "min_vote", min_vote,
        "max_lines", max_lines,
        "granularity", granularity,
        "min_ratio", min_ratio,
        "max_angle", max_angle_deg,
        "min_width", min_width,
        "max_width", max_width
    )) return false;
    max_angle = max_angle_deg * M_PI / 180.0;
    
    // estimate size of Hough space. Mostly copied from hough 3d library, with size estimated as worst case point cloud bounding box
    auto volume = cloud->volume();
    auto min_p = Eigen::Vector<float, 3>{-volume[0]/2.0f, -volume[1]/2.0f, 0} * 1000.0f;
    auto max_p = Eigen::Vector<float, 3>{volume[0]/2.0f, volume[1]/2.0f, volume[2]} * 1000.0f;
    hough = new Hough(min_p, max_p, 0, granularity);
    info("Configured Hough space with volume of ", volume[0], "m x ", volume[1], "m x ", volume[2], "m, and resolution of ", hough->dx, "mm");

    return true;
}


void LineFinder::stop_impl()
{}


static void orthogonal_LSQ(const Eigen::Matrix<float, 3, Eigen::Dynamic> &points, Eigen::Vector<float, 3>& a, Eigen::Vector<float, 3>& b, Eigen::Vector<float, 3>& c)
{
    // anchor point is mean value
    a = points.rowwise().mean();

    // compute scatter matrix ...
    auto centered = points.colwise() - a;
    auto scatter = centered * centered.adjoint();

    // ... and its eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3, 3>> eig(scatter);
    Eigen::Matrix<float, 3, 3> eigvecs = eig.eigenvectors();

    // we need eigenvector to largest eigenvalue
    // libeigen yields it as LAST column
    b = eigvecs.col(2);
    c = eigvecs.col(1);
}


static std::pair<float, float> dimensions(const Eigen::Matrix<float, 3, Eigen::Dynamic> &points, const Eigen::Vector<float, 3> &anchor, const Eigen::Vector<float, 3> &dir, const Eigen::Vector<float, 3> &normal)
{
    const auto shifted = points.colwise() - anchor;

    const Eigen::VectorXf lengths = dir.transpose() * shifted;
    const float length_max = lengths.maxCoeff();
    const float length_min = lengths.minCoeff();

    const Eigen::VectorXf widths = normal.transpose() * shifted;
    const float width_max = widths.maxCoeff();
    const float width_min = widths.minCoeff();

    return {(length_max - length_min) / 2.0f, (width_max - width_min) / 2.0f};
}


std::vector<Line> LineFinder::hough3d(
    const Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>> &points,
    Eigen::Map<Eigen::Vector<int8_t, Eigen::Dynamic>>& ids)
{
    // Perform hough transform iteratively
    // This vector tracks positions in original point cloud as lines get iteratively removed
    Eigen::VectorXi index_map = Eigen::VectorXi::LinSpaced(points.cols(), 0, points.cols()-1);

    hough->add(points);

    std::vector<Eigen::Index> Y; // points close to line
    double l, w;
    int nlines = 0;
    std::vector<Line> out; // Identified lines
    do
    {
        Eigen::Vector3f a; // anchor point of line
        Eigen::Vector3f b; // direction of line
        Eigen::Vector3f c; // perpendicular of line

        // Get initial line estimate
        unsigned nvotes = hough->getLine(a, b);

        // Get the highest voted line
        Y = indicesCloseToLine(points(Eigen::all, index_map), a, b, hough->dx);

        // Refine line
        orthogonal_LSQ(points(Eigen::all, index_map(Y)), a, b, c);

        // Refine inliers
        Y = indicesCloseToLine(points(Eigen::all, index_map), a, b, hough->dx);
        nvotes = Y.size();
        if (nvotes < min_vote)
            // Vote threshold not met, exit
            break;

        // Refine line again?
        orthogonal_LSQ(points(Eigen::all, index_map(Y)), a, b, c);

        std::tie(l, w) = dimensions(points(Eigen::all, index_map(Y)), a, b, c);
        
        // a += center;
        auto ratio = l / w;
        auto cos_theta = std::abs(b(2));

        if (
            (w*2.0 <= max_width) &&
            (w*2.0 >= min_width) &&
            (ratio >= min_ratio) &&
            (cos_theta <= std::sin(max_angle)))
        {
            // Add this line
            Line line;
            memcpy(line.anchor, a.data(), 3*sizeof(float));
            memcpy(line.dir, (b*l).eval().data(), 3*sizeof(float));
            memcpy(line.normal, (c*w).eval().data(), 3*sizeof(float));
            out.push_back(line);
            // Mark points belonging to this line, using index map to point back to original indices
            ids(index_map(Y)).fill(nlines);

            ++nlines;
        }

        // Remove this line to prepare to get the next highest voted
        hough->subtract(points(Eigen::all, index_map(Y)));
        auto idx_new = removePoints(index_map.size(), Y);
        index_map = index_map(idx_new).eval();

    } while ((index_map.size() > 1) &&
             ((max_lines == 0) || (max_lines > nlines)));

    hough->reset();

    return out;
}


bool _flag_given(const std::vector<std::string>& args, const std::string& flag)
{
    for(auto& x : args)
    {
        if(x == flag) return true;
    }
    return false;
}


void save_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    // Initialize
    auto taskptr = (LineFinder*)(task);
    const auto filename = args[0];
    std::ofstream file(filename, std::ios::out | std::ios::binary);
    if(!file.is_open())
    {
        out << "Failed to open " << filename << std::endl;
        return;
    }

    bool ready = !_flag_given(args, "--wait");
    LineFinder::update_ptr_const_type update = taskptr->acquire();
    // Wait for valid data if not ready
    while(!ready)
    {
        update = taskptr->acquire();
        ready = update->data.valid;
    }

    // Pack data
    const uint32_t n = static_cast<uint32_t>(update->data.pointcloud->data.n_valid);
    const uint32_t w = static_cast<uint32_t>(CameraWrapper::Width);
    const uint32_t h = static_cast<uint32_t>(CameraWrapper::Height);
    const uint8_t idx = update->data.selected_line;

    file.write((char*)&update->data.lines[idx].anchor, 3*sizeof(float));
    file.write((char*)&update->data.lines[idx].dir, 3*sizeof(float));
    file.write((char*)&n, sizeof(uint32_t));
    file.write((char*)&update->data.pointcloud->data.cloud, n * 3 * sizeof(float));
    file.write((char*)&w, sizeof(uint32_t));
    file.write((char*)&h, sizeof(uint32_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.left_img->data, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.right_img->data, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.confidence, w*h*sizeof(uint8_t));
    file.write((char*)&update->data.pointcloud->data.disparity->data.disparity, w*h*sizeof(int16_t));
    auto n_rejects = update->data.n_lines - 1;
    file.write((char*)&n_rejects, sizeof(uint8_t));
    for(unsigned i = 0; i < std::min(static_cast<unsigned>(update->data.n_lines), MAX_LINES); ++i)
    {
        if(i != idx)
        {
            file.write((char*)&update->data.lines[i].anchor, 3*sizeof(float));
            file.write((char*)&update->data.lines[i].dir, 3*sizeof(float));
        }
    }
    file.write((char*)&update->data.line_ids, n * sizeof(int8_t));

}


void report_line_detect(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    bool ready = !_flag_given(args, "--wait");
    bool json = _flag_given(args, "--json");
    auto taskptr = (LineFinder*)(task);
    LineFinder::update_ptr_const_type update = taskptr->acquire();
    // Wait for valid data if not ready
    while(!ready)
    {
        update = taskptr->acquire();
        ready = update->data.valid;
    }

    if(!update->data.valid)
    {
        if(json)
        {
            out << "{\"valid\": false}";
        } else {
            out << "No perch detected." << std::endl;
        }
        return;
    }

    auto idx = update->data.selected_line;
    auto a = Eigen::Map<const Eigen::Vector3f>(update->data.lines[idx].anchor);
    auto d = Eigen::Map<const Eigen::Vector3f>(update->data.lines[idx].dir);
    float angle_z = std::acos(d(2) / d.norm()); // Angle in radians to z axis 
    const Eigen::Vector3f d_xy = {d(0), d(1), 0};
    float angle_y = std::acos(d_xy(1) / d_xy.norm()); // Angle in radians to y axis 

    if(json)
    {
        out << "{";
        out << "\"valid\":true,";
        out << "\"distance\":" << a[2] << ",";
        out << "\"anchor\":[" << a[0] << "," << a[1] << "," << a[2] << "], ";
        out << "\"angle_cam_plane\":" << 90.0 - (angle_z * 180.0 / M_PI) << ",";
        out << "\"angle_cam_vertical\":" << angle_y * 180.0 / M_PI << ",";
        out << "\"n_lines\":" << (int)update->data.n_lines;
        out << "}" << std::endl;
    } else {
        out << "Perch found." << std::endl;
        out << "\tDistance: " << a[2] << " mm" << std::endl;
        out << "\tAnchor Point: " << a << std::endl;
        out << "\tAngle From Camera Plane: " << 90.0 - (angle_z * 180.0 / M_PI) << " degrees" << std::endl;
        out << "\tAngle From Camera Vertical: " << angle_y * 180.0 / M_PI << " degrees" << std::endl;
        out << "\tTotal Candidate Lines: " << (int)update->data.n_lines << std::endl;
    }
}