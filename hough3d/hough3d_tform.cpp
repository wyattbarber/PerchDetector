#include "hough3d_tform.hpp"


void orthogonal_LSQ(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, Eigen::Vector<double, 3>& a, Eigen::Vector<double, 3>& b, Eigen::Vector<double, 3>& c)
{
    // anchor point is mean value
    a = points.rowwise().mean();

    // compute scatter matrix ...
    auto centered = points.colwise() - a;
    auto scatter = centered * centered.adjoint();

    // ... and its eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> eig(scatter);
    Eigen::Matrix<double, 3, 3> eigvecs = eig.eigenvectors();

    // we need eigenvector to largest eigenvalue
    // libeigen yields it as LAST column
    b = eigvecs.col(2);
    c = eigvecs.col(1);
}


std::vector<Line> hough3d(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, 
    Hough& hough,
    size_t min_vote, 
    size_t maxlines, 
    double min_width, 
    double max_width, 
    double min_ratio,
    double max_angle)
{
    // Perform hough transform iteratively
    hough.add(points);

    std::vector<size_t> Y; // points close to line
    double l, w;
    unsigned int nvotes;
    int nlines = 0;
    std::vector<Line> out; // Identified lines
    do
    {
        Eigen::Vector<double, 3> a; // anchor point of line
        Eigen::Vector<double, 3> b; // direction of line
        Eigen::Vector<double, 3> c; // perpendicular of line

        hough.subtract(points(Eigen::all, Y)); // do it here to save one call

        // Get the highest voted line
        Y = indicesCloseToLine(points, a, b, hough.dx);

        // Refine line
        orthogonal_LSQ(points(Eigen::all, Y), a, b, c);

        // Refine inliers ?
        Y = indicesCloseToLine(points, a, b, hough.dx);
        nvotes = Y.size();
        if (nvotes < min_vote)
            // Vote threshold not met, exit
            break;

        // Refine line again?
        orthogonal_LSQ(points(Eigen::all, Y), a, b, c);

        // a += shift;

        std::tie(l, w) = dimensions(points(Eigen::all, Y), a, b, c);

        auto ratio = l / w;
        auto cos_theta = std::abs(b(2));

        if (
            (w*2.0 <= max_width) &&
            (w*2.0 >= min_width) &&
            (ratio >= min_ratio) &&
            (cos_theta <= std::sin(max_angle)))
        {
            out.push_back(std::make_tuple(a, b, c));
            ++nlines;
        }

        // // Remove this line to prepare to get the next highest voted
        // points = removePoints(points, Y);

    } while ((points.cols() > 1) &&
             ((maxlines == 0) || (maxlines > nlines)));

    return out;
}


auto line_inliers(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b)
{
    Eigen::Vector<double, 3> minP = points.rowwise().minCoeff();
    Eigen::Vector<double, 3> maxP = points.rowwise().maxCoeff();
    auto d = (maxP - minP).norm();
    auto opt_dx = d / 64.0;

    return indicesCloseToLine(points, a, b, opt_dx);
}


std::pair<double, double> dimensions(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, const Eigen::Vector<double, 3> &anchor, const Eigen::Vector<double, 3> &dir, const Eigen::Vector<double, 3> &normal)
{
    const auto shifted = points.colwise() - anchor;
    double length_max = 0.0, length_min = 0.0, width_max = 0.0, width_min = 0.0;
    for (int i = 0; i < shifted.cols(); ++i)
    {
        double l = shifted.col(i).dot(dir);
        length_max = std::max(l, length_max);
        length_min = std::min(l, length_min);
        double w = shifted.col(i).dot(normal);
        width_max = std::max(w, width_max);
        width_min = std::min(w, width_min);
    }
    return {(length_max - length_min) / 2.0, (width_max - width_min) / 2.0};
}
