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
    size = image.rows*image.cols;
    std::cout << "Opening " << file << std::endl;
    fd = open(file, O_RDWR);
    if(fd == -1)
    {
        std::cerr << "Failed to open " << file << ": " << strerror(errno) << std::endl;
        return;
    }
    // if(ftruncate(fd, size) == -1);
    // {
    //     std::cerr << "Failed to set file size " << size << ": " << strerror(errno) << std::endl;
    //     return;
    // }
    // Now create the map
    map = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if(map == MAP_FAILED)
    {
        close(fd);
        std::cerr << "Failed to map memory: " << ": " << strerror(errno) << std::endl;
        return;
    }
    valid = true;
}


void ImageSender::stop()
{
    valid = false;
    if(munmap(map, size) < 0)
    {
        std::cerr << "Failed to unmap memory" << std::endl;
    }
    close(fd);
}


void ImageSender::transmit()
{
    ((uint8_t*)map)[0] = 0xFF; // lock
    uint8_t* start = ((uint8_t*)map)+1;
    memcpy((void*)start, image.data, size);
    ((uint8_t*)map)[0] = 0x00; // unlock
}