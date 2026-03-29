#include "CameraSim.hpp"
#include <opencv2/core/eigen.hpp>
#include <chrono>


using namespace Eigen;


void CameraSimulator::step()
{
    tick();

    // Initialize image with noise in the range 0-25
    background.setRandom(Height, Width);
    background = background.unaryExpr([](uint8_t x){ return x % 25; }).cast<uint8_t>();

    // Set block in middle of frame to white
    unsigned mid_x = Width / 2;
    unsigned mid_y = Height / 2;
    background.block<block_height, block_width>(
        mid_y - (block_height/2),
        right ? mid_x - (block_width/2) - 50 : mid_x - (block_width/2)
    ) = block;

    // Update output data
    cv::eigen2cv(background, cv_out);
    swap_data(*reinterpret_cast<uint8_t(*)[Width*Height]>(cv_out.data));
}

