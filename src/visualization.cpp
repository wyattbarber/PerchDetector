#include <visualization.hpp>
#include <Depth.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <errno.h>

std::unique_ptr<ImageSender> visualization::image, visualization::depth;
std::unique_ptr<visualization::Params> visualization::params;


int visualization::setup(
    ArgParser& args, 
    std::shared_ptr<CameraWrapper> camera_left, 
    std::shared_ptr<CameraWrapper> camera_right)
{
    image = make_sender<CameraWrapper::dtype>(
        args.image_map_file(), camera_left->get_width(), camera_left->get_height()
    );
    image->start();
    if(!image->opened())
    {
        std::cerr << "Failed to setup image communication channel" << std::endl;
        return -1;
    }

    depth = make_sender<int16_t>(
        args.depth_map_file(), camera_left->get_width(), camera_left->get_height()
    );
    depth->start();
    if(!depth->opened())
    {
        std::cerr << "Failed to setup depth communication channel" << std::endl;
        return -1;
    }

    params = std::make_unique<Params>(args.parameter_map_file());
    params->start();
    if(!params->opened())
    {
        std::cerr << "Failed to setup parameter channel" << std::endl;
        return -1;
    }

    return 0;
}

void visualization::teardown()
{
    params->stop();
    image->stop();
    depth->stop();
}


void visualization::Params::start()
{
    // Open the file and ensure it has enough space
    std::cout << "Opening " << file << std::endl;
    // Open file, with permission to create it and resize it, and set mode to allow read/write for self and read for all users
    fd = open(file, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    if(fd == -1)
    {
        std::cerr << "Failed to open " << file << ": " << strerror(errno) << std::endl;
        return;
    }
    // Set the file size
    if(ftruncate(fd, size) == -1)
    {
        std::cerr << "Failed to set file size to  " << size << ": " << strerror(errno) << std::endl;
        return;
    }
    // Now create the map
    map = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(map == MAP_FAILED)
    {
        close(fd);
        std::cerr << "Failed to map memory: " << strerror(errno) << std::endl;
        return;
    }
    valid = true;
}


void visualization::Params::stop()
{
    valid = false;
    if(munmap(map, size) < 0)
    {
        std::cerr << "Failed to unmap memory: " << strerror(errno)  << std::endl;
    }
    close(fd);
    if(remove(file) != 0)
    {
        std::cerr << "Failed to delete " << file << ": " << strerror(errno)  << std::endl;
    }
}


bool visualization::Params::check()
{
    bool out = false;
    if(valid)
    {
        bool _lock = ((uint8_t*)map)[0] != 0;
        out = (!_lock) && _lock_prev;
        _lock_prev = _lock;
    }
    return out;
}