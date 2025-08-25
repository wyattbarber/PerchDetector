#include <ImageSender.hpp>
#include <Logging.hpp>
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
    Logger::instance() << "Opening " << file << std::endl;
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
    // Create a matrix mapped to the mapped array
    ((uint8_t*)map)[0] = 0x00;
    ((uint8_t*)map)[1] = static_cast<uint8_t>(cvtype);
    ((uint8_t*)map)[2] = static_cast<uint8_t>(height & 0x00FF);
    ((uint8_t*)map)[3] = static_cast<uint8_t>((height & 0xFF00) >> 8);
    ((uint8_t*)map)[4] = static_cast<uint8_t>(width & 0x00FF);
    ((uint8_t*)map)[5] = static_cast<uint8_t>((width & 0xFF00) >> 8);
    image = cv::Mat(height, width, cvtype, (void*)((uint8_t*)map+6));
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
    if(remove(file) != 0)
    {
        std::cerr << "Failed to delete " << file << ": " << strerror(errno)  << std::endl;
    }
}


bool ImageSender::opened()
{
    return valid;
}


void ImageSender::acquire()
{
    ((uint8_t*)map)[0] = 0xFF;
}


void ImageSender::release()
{
    ((uint8_t*)map)[0] = 0x00;
}


cv::Mat* ImageSender::get_image()
{
    return &image;
}