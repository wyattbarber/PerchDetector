#pragma once

#include <cxxabi.h>
#include <sstream>
#include <chrono>
#include <thread>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <cerrno>


template<class T, typename C, typename CF>
bool DataMapper<T, C, CF>::start_impl(){ 
    if(!task->is_alive())
    {
        error("Cannot start streamer if producer is not running.");
        return false;
    }
    latest = task->acquire();
    return true;
}


template<class T, typename C, typename CF>
void DataMapper<T, C, CF>::stop_impl()
{
    if(running)
    {
        info("Mapped file still open, unmapping and closing.");
        // Wait for in progress write to stop
        while(mapping){}
        if(munmap(map, sizeof(C)+1))
        {
            error("Error unmapping file: ", std::strerror(errno));
        }
        std::fclose(map_file);
        info("Unmapped and closed file.");
    }
}


template<class T, typename C, typename CF>
void DataMapper<T, C, CF>::step()
{
    if(is_alive() & running)
    {
        if(latest->stale) // Memory map is opened and new data is available
        {
            mapping = true;
            latest = task->acquire();
            char* dst = reinterpret_cast<char*>(map)+1;
            converter((void*)dst, latest->data);
            reinterpret_cast<char*>(map)[0] = 0xFF;
            reinterpret_cast<char*>(map)[0] = 0x00;
            mapping = false;
        }
    }
}


template<class T>
void map_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    auto t = (DataMapper<T>*)task;

    if(args.size() < 1)
    {
        out << "File name for mapping is required." << std::endl;
        return;
    }
    
    t->map_file = std::fopen(args[0].c_str(), "w+");
    if(!t->map_file)
    {
        out << "Failed to open file " << args[0] << ": " << std::strerror(errno) << std::endl;
        return;
    }
    if(ftruncate(fileno(t->map_file), sizeof(typename T::value_type)+1))
    {
        out << "Failed to set file size to " << sizeof(typename T::value_type)+1 << ": " << std::strerror(errno) << std::endl;
        std::fclose(t->map_file);
        return;
    }
    t->map = mmap(0, sizeof(typename T::value_type)+1, PROT_READ | PROT_WRITE, MAP_SHARED, fileno(t->map_file), 0);
    if(t->map == MAP_FAILED)
    {
        out << "Failed to create memory map: " << std::strerror(errno) << std::endl;
        std::fclose(t->map_file);
        return;
    }

    // Set initial data and start updates    
    char* dst = reinterpret_cast<char*>(t->map)+1;
    memcpy((void*)dst, (void*)(&t->latest->data), sizeof(typename T::value_type)); 
    t->running = true;
    t->info("Mapped data stream to ", args[0]);
}


template<class T>
void unmap_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    auto t = (DataMapper<T>*)task;
    t->info("Unmapping data stream");

    t->running = false;
    // Wait for in progress write to stop
    while(t->mapping){}

    if(munmap(t->map, sizeof(typename T::value_type)+1))
    {
        out << "Error unmapping file: " << std::strerror(errno) << std::endl;
    }
    std::fclose(t->map_file);
    out << "Unmapping done." << std::endl;
}


template<class T>
void data_dims(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    auto t = (T*)task;    
    for(size_t d : t->dims())
    {
        out << d << " ";
    }
    out << std::endl;
}