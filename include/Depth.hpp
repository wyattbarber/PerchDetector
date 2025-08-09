#pragma once

#include <CameraWrapper.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>


/** Manages getting grayscale frames from two cameras and maintaining a buffer of depth maps.
*/
class DepthCamera
{
public:
    typedef float dtype; /// Datatype used for depth of each pixel
    static constexpr auto cvtype = cv::DataType<dtype>::type; // OpenCV type ID of pixel depth values

    /** Construct a new depth camera
    
    @param left Left camera wrapper
    @param right Right camera wrapper
    */  
    DepthCamera(std::shared_ptr<CameraWrapper> left, std::shared_ptr<CameraWrapper> right) : 
        left(left),
        right(right),
        left_intr(3,3,CV_64F,(void*)_left_intrinsic), 
        right_intr(3,3,CV_64F,(void*)_right_intrinsic), 
        left_dist(1,5,CV_64F,(void*)_left_distortion), 
        right_dist(1,5,CV_64F,(void*)_right_distortion), 
        R(3,3,CV_64F,(void*)_R), 
        T(3,1,CV_64F,(void*)_T)
    {
        stereo = cv::StereoSGBM::create(
            	0, // minDisparity
                256, // numDisparities
                5, // blockSize
                0, // P1
                0, // P2
                -1, // disp12MaxDiff
                31, // preFilterCap
                20, // uniquenessRatio
                0, // speckleWindowSize
                0 // speckleRange
            );
        _stereo_locked = false;
        locked_idx = 0;
    }

    /** Locks the most recent depth map,
    preventing it from being overwritten,
    */
    void lock(){ locked_idx = latest_idx; }

    /** Release the previously locked depth map to be overwritten.
    */
    void unlock(){ locked_idx = -1; }

    /** Collects one new disparity map.
    */
    void update();

    /** Gets a pointer to the most recently 
    accuired and locked disparity map. 
    
    @return Pointer to disparity data.
    */
    const cv::Mat* disparity(){ return &_disparity[locked_idx]; }

    /** Computes depth for the latest data.
    
    @return Depth image
    */
    cv::Mat depth();

    /** Initalizes data arrays.
    
    Should be called after cameras are initialized and their shape data
    is available.
    */
    void initialize();

    /** Changes the parameters for StereoSGBM.
     
    */
    void set_params(int	minDisparity,
                    int	numDisparities,
                    int	blockSize,
                    int	P1,
                    int	P2,
                    int	disp12MaxDiff,
                    int	preFilterCap,
                    int	uniquenessRatio,
                    int	speckleWindowSize,
                    int	speckleRange);

protected:
    std::shared_ptr<CameraWrapper> left, right;
    cv::Ptr<cv::StereoSGBM> stereo;
    bool _stereo_locked;

    // Data buffer
    static constexpr size_t N = 4;
    int locked_idx;
    size_t latest_idx;
    cv::Mat _disparity[N];
    cv::Mat rect_l, rect_r;

    struct disp_conv
    {
        float M;

        void operator()(float& p, const int* idx) const
        {
            if(std::abs(p) < 1e-3)
                p = 0.0;
            else
                p = M / (p + std::numeric_limits<float>::epsilon());
        }
    };
    struct disp_conv converter;

    // Camera parameters (distance units are cm)
    cv::Mat mapl1, mapl2, mapr1, mapr2, Q;
    const cv::Mat left_intr, right_intr, left_dist, right_dist, R, T;
    static constexpr double _left_intrinsic[] = {581.0291623263779, 0, 402.6076033373866, 0, 581.0291623263779, 321.8633871605039, 0, 0, 1};
    static constexpr double _left_distortion[] = {0.1070047468526379, -0.275348362674357, 0, 0, 0, 0, 0, -0.07797686695310302, 0, 0, 0, 0, 0, 0};
    static constexpr double _right_intrinsic[] = {581.0291623263779, 0, 406.3044950461705, 0, 581.0291623263779, 317.2505827088175, 0, 0, 1};
    static constexpr double _right_distortion[] = {0.07495053383740824, -0.0534409532722507, 0, 0, 0, 0, 0, 0.2097640304756714, 0, 0, 0, 0, 0, 0};
    static constexpr double _R[] = {0.9967761808393969, -0.07640703898491494, -0.02447876027873495, 0.07639049952632984, 0.9970766745111561, -0.001611436592591298, 0.02453032599342867, -0.0002637031130498094, 0.9996990514986619}; 
    static constexpr double _T[] = {-5.516592004583581, -0.2893022456287973, 0.0009606096523024099}; 
};