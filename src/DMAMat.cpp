#include <DMAMat.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <errno.h>


bool DMAMat::set(const libcamera::FrameBuffer* fb, int cvtype)
{
    if((map == nullptr) || (fb->planes()[0].fd.get() != fd))
    {
        // Map is unset or file descriptor has changed
        if(map != nullptr)
        {
            std::cout << "Resetting mapped matrix" << std::endl;
            if(!unmap()) return false;
        }
        else
        {
            std::cout << "Initializing mapped matrix" << std::endl;
        }

        fd = fb->planes()[0].fd.get();
        size = fb->planes()[0].length;        
        std::cout << "Mapping" << std::endl;
        map = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if(map == MAP_FAILED)
        {
            close(fd);
            std::cerr << "Failed to map memory: " << strerror(errno) << std::endl;
            return false;
        }
        else
        {
            std::cout << "Memory mapping succeeded" << std::endl;
        }

        // Map memory to matrix
        std::cout << "Mapping " << size << " bytes to " << height 
            << " x " << width << " matrix of type " << cvtype << "." << std::endl;
        *matrix = cv::Mat(height, width, cvtype, map);
    }

    // No change is needed
    return true;
}


bool DMAMat::unmap()
{
    if(munmap(map, size) < 0)
    {
        std::cerr << "Failed to unmap memory: " << strerror(errno)  << std::endl;
        return false;
    }
    return true;
}


std::shared_ptr<cv::Mat> DMAMat::get()
{
    return matrix;
}
