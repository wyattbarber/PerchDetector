#include "hough3d_tform.hpp"


std::pair<double, double> orthogonal_LSQ(const Eigen::Matrix<double, Eigen::Dynamic, 3> &points, Vector3d* a, Vector3d* b, Vector3d* c){
  // anchor point is mean value
  *a = points.mean();

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


std::vector<Line> hough3d(const Eigen::Matrix<double, Eigen::Dynamic, 3> &points, 
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

    Eigen::Matrix<double, Eigen::Dynamic, 3> Y; // points close to line
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
        pointsCloseToLine(X, a, b, hough.dx, Y);

        // Refine line
        orthogonal_LSQ(Y, &a, &b, &c);
        if (l == 0.0)
            break;
        
        // Refine inliers ?
        pointsCloseToLine(X, a, b, hough.dx, Y);
        nvotes = Y.cols();
        if (nvotes < min_vote)
            // Vote threshold not met, exit
            break;

        // Refine line again?
        orthogonal_LSQ(Y, &a, &b, &c);
        if (l == 0.0)
            break;
        
        // a = a + X.shift;

        l = projected_length(Y, {b.x, b.y, b.z});
        w = projected_length(Y, {c.x, c.y, c.z});
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
        removePoints(X, Y);
    } while ((X.cols() > 1) &&
             ((maxlines == 0) || (maxlines > nlines)));

    hough.reset();

    return out;
}


auto line_inliers(const Eigen::Matrix<double, 3, Eigen::Dynamic> &points, const Eigen::Vector<double, 3> &a, const Eigen::Vector<double, 3> &b)
{
    HoughPointCloud X, Y;
    for(int i = 0; i < points.cols(); ++i)
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


double projected_length(const Eigen::Matrix<double, Eigen::Dynamic, 3>& points, const Eigen::Vector<double, 3>& dir)
{
    double out = 0.0;
    for(int i = 0; i < points.cols(); ++i)
    {
        double x = points.col(i).dot(dir);
        if(abs(x) > out) out = x;
    }
    return out;
}