#include "CameraSim.hpp"
#include <opencv2/core/eigen.hpp>
#include <chrono>


using namespace Eigen;


void CameraSimulator::step()
{
    tick();

    auto latest = manager->acquire();
    swap_data(right ? latest->data.right : latest->data.left);
}


void CameraSimManager::set_source_file(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    auto tp = (CameraSimManager*)task;

    if(args.size() < 1)
    {
        out << "Need input filename." << std::endl;
        return;
    }
    const auto& filename = args[0];

    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if(!file.is_open())
    {
        out << "Failed to open " << filename << std::endl;
        return;
    }

    uint32_t n, w, h;
    file.seekg(6*sizeof(float));
    file.read((char*)&n, sizeof(uint32_t));
    file.seekg(n*3*sizeof(float), std::ios::cur);
    file.read((char*)&w, sizeof(uint32_t));
    file.read((char*)&h, sizeof(uint32_t));

    if((w != Width) || (h != Height))
    {
        out << "Dimensions stored in file are incorrect. Expected " << Width << " x " << Height;
        out << ", got " << w << " x " << h << std::endl;
        return;
    }

    auto next = tp->allocate_next();
    CameraSimulator::value_type buffer;
    file.read((char*)buffer, Width*Height);
    memcpy((void*)next->data.left, buffer, Width*Height);
    file.read((char*)buffer, Width*Height);
    memcpy((void*)next->data.right, buffer, Width*Height);
    tp->swap_data();

    out << "Loaded new data file." << std::endl;
}