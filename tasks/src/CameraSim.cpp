#include "CameraSim.hpp"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <chrono>


using namespace Eigen;



bool CameraSimulator::start_impl()
{
    return true;
}


void CameraSimulator::stop_impl()
{

}


void CameraSimulator::step()
{
    if(!is_alive()) return;
    tick();

    // Initialize image with noise in the range 0-25
    Matrix<uint8_t, Dynamic, Dynamic> img = Matrix<uint8_t, Dynamic, Dynamic>::Random(Height, Width)
        .unaryExpr([](uint8_t x){ return x % 25; }).cast<uint8_t>();

    // Set block in middle of frame to white
    unsigned mid_x = Width / 2;
    unsigned mid_y = Height / 2;
    Matrix<uint8_t, Dynamic, Dynamic> block = Matrix<uint8_t, Dynamic, Dynamic>::Ones(block_height, block_width) * 255;
    img.block<block_height, block_width>(
        mid_y - (block_height/2),
        mid_x - (block_width/2)
    ) = block;

    // Update output data
    cv::Mat out;
    cv::eigen2cv(img, out);
    swap_data(*reinterpret_cast<uint8_t(*)[Width*Height]>(out.data));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

