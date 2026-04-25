#include "hough3d_tform.hpp"



std::pair<double, double> orthogonal_LSQ(const HoughPointCloud &pc, Vector3d* a, Vector3d* b, Vector3d* c){

  // anchor point is mean value
  *a = pc.meanValue();

  // copy points to libeigen matrix
  Eigen::MatrixXf points = Eigen::MatrixXf::Constant(pc.points.size(), 3, 0);
  for (int i = 0; i < points.rows(); i++) {
    points(i,0) = pc.points.at(i).x;
    points(i,1) = pc.points.at(i).y;
    points(i,2) = pc.points.at(i).z;
  }

  // compute scatter matrix ...
  Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
  Eigen::MatrixXf scatter = (centered.adjoint() * centered);

  // ... and its eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(scatter);
  Eigen::MatrixXf eigvecs = eig.eigenvectors();

  // we need eigenvector to largest eigenvalue
  // libeigen yields it as LASdouble column
  b->x = eigvecs(0,2); b->y = eigvecs(1,2); b->z = eigvecs(2,2);
  c->x = eigvecs(0,1); c->y = eigvecs(1,1); c->z = eigvecs(2,1);

  // Return the first and second eigenvalues, corresponding to the length and width of the detected line
  auto dev = (eig.eigenvalues() / (points.rows()-1)).cwiseSqrt().eval();
  return {dev(2), dev(1)};
}


std::vector<Line> hough3d(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, 
    size_t min_vote, 
    size_t maxlines, 
    size_t granularity, 
    double min_width, 
    double max_width, 
    double min_ratio,
    double max_angle)
{
    // Granularity options
    int num_directions[7] = {12, 21, 81, 321, 1281, 5121, 20481};

    // Assemble point cloud and get bounding box
    HoughPointCloud X;
    Vector3d minP, maxP, minPshifted, maxPshifted;
    for(size_t i = 0; i < points.cols(); ++i)
    {
        X.points.push_back(
            Vector3d(points(0,i), points(1,i), points(2,i))
        );
    }
    X.getMinMax3D(&minP, &maxP);
    double d = (maxP - minP).norm();
    X.shiftToOrigin();
    X.getMinMax3D(&minPshifted, &maxPshifted);

    // estimate size of Hough space
    double opt_dx = d / 64.0;
    double num_x = floor(d / opt_dx + 0.5);
    double num_cells = num_x * num_x * num_directions[granularity];

    // Perform hough transform iteratively
    Hough hough(minPshifted, maxPshifted, opt_dx, granularity);
    hough.add(X);

    HoughPointCloud Y; // points close to line
    double l, w;
    unsigned int nvotes;
    int nlines = 0;
    std::vector<Line> out; // Identified lines
    do
    {
        Vector3d a; // anchor point of line
        Vector3d b; // direction of line
        Vector3d c; // perpendicular of line

        hough.subtract(Y); // do it here to save one call
        
        // Get the highest voted line
        nvotes = hough.getLine(&a, &b);
        X.pointsCloseToLine(a, b, opt_dx, &Y);

        // Refine line
        orthogonal_LSQ(Y, &a, &b, &c);
        if (l == 0.0)
            break;
        
        // Refine inliers ?
        X.pointsCloseToLine(a, b, opt_dx, &Y);
        nvotes = Y.points.size();
        if (nvotes < min_vote)
            // Vote threshold not met, exit
            break;

        // Refine line again?
        orthogonal_LSQ(Y, &a, &b, &c);
        if (l == 0.0)
            break;
        
        a = a + X.shift;

        auto inliers = Y.mat();
        l = projected_length(inliers, {b.x, b.y, b.z});
        w = projected_length(inliers, {c.x, c.y, c.z});
        auto ratio = l / w;
        auto cos_theta = std::abs(b.z);

        if(
            (w <= max_width) && 
            (w >= min_width) && 
            (ratio >= min_ratio) && 
            (cos_theta <= std::sin(max_angle))
        )
        {
            out.push_back(std::make_tuple(
                Eigen::Vector<double, 3> {a.x, a.y, a.z},
                Eigen::Vector<double, 3> {b.x * l, b.y * l, b.z * l},
                Eigen::Vector<double, 3> {c.x * w, c.y * w, c.z * w}
            ));

            ++nlines;
        }
        
        // Remove this line to prepare to get the next highest voted
        X.removePoints(Y);
    } while ((X.points.size() > 1) &&
             ((maxlines == 0) || (maxlines > nlines)));

    return out;
}


auto line_inliers(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b)
{
    HoughPointCloud X, Y;
    for(size_t i = 0; i < points.cols(); ++i)
    {
        X.points.push_back(
            Vector3d(points(0,i), points(1,i), points(2,i))
        );
    }
    Vector3d minP, maxP;
    X.getMinMax3D(&minP, &maxP);
    double d = (maxP - minP).norm();

    std::vector<size_t> out;
    X.pointsCloseToLine(
        Vector3d(a(0), a(1), a(2)), 
        Vector3d(b(0), b(1), b(2)), 
        d / 64.0, out
    );
    return out;
}


double projected_length(const Eigen::Matrix<double, 3, Eigen::Dynamic>& points, const Eigen::Vector<double, 3>& dir)
{
    double out = 0.0;
    for(size_t i = 0; i < points.cols(); ++i)
    {
        double x = points.col(i).dot(dir);
        if(abs(x) > out) out = x;
    }
    return out;
}