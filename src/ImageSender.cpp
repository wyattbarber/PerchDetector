#include <ImageSender.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <iostream>
#include <errno.h>


void ImageSender::start()
{
    // Open the file and ensure it has enough space
    std::cout << "Opening " << file << std::endl;
    fd = open(file, O_RDWR);
    if(fd == -1)
    {
        std::cerr << "Failed to open " << file << ": " << strerror(errno) << std::endl;
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
    // Create a matrix mapped to the mapped array
    ((uint8_t*)map[0]) = 0x00;
    ((uint8_t*)map[1]) = static_cast<uint8_t>(cvtype);
    image = cv::Mat(height, width, (void*)((uint8_t*)map+2), cvtype);
    valid = true;
}


void ImageSender::stop()
{
    valid = false;
    if(munmap(map, size) < 0)
    {
        std::cerr << "Failed to unmap memory: " << strerror(errno)  << std::endl;
    }
    close(fd);
}


bool ImageSender::opened()
{
    return valid;
}


void ImageSender::acquire()
{
    ((uint8_t*)map[0]) = 0xFF;
}


void ImageSender::release()
{
    ((uint8_t*)map[0]) = 0x00;
}


cv::Mat* ImageSender::get_image()
{
    return &image;
}