#pragma once

#include <cxxabi.h>
#include <sstream>
#include <chrono>
#include <thread>


template<class T, typename D>
bool DataEncoder<T,D>::start_impl(){ 
    if(!task->is_alive())
    {
        error("Cannot start streamer if producer is not running.");
        return false;
    }

    size = task->size();
    info("Allocating streaming buffer of ", size, " bytes.");
    buffer = (D*)malloc(size*sizeof(D));
    return true;
}


template<class T, typename D>
void DataEncoder<T,D>::stop_impl()
{
    info("Freeing streaming buffer.");
    free(buffer);
}


template<class T, typename D>
void DataEncoder<T,D>::step()
{
    if(run_stream)
    {
        streaming = true;
        size_t size = task->size();
        memcpy((void*)buffer, (void*)task->acquire(), size*sizeof(D));
        task->release();
        
        // Dump bytes of all data to output
        for(size_t i = 0; i < size; ++i)
        {
            if(!run_stream) break; // Check for exit mid frame
            using bytearray_t = unsigned char[sizeof(D)];
            bytearray_t* data = reinterpret_cast<bytearray_t*>(&(buffer[i]));
            for(size_t j = 0; j < sizeof(D); ++j)
            {
                (*out) << data[j];
            } 
        }
        tick();
    }
    else
    {
        streaming = false;
    }
}


template<class T, typename D>
void print_datatype(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    int status;
    std::string name = typeid(D).name();
    char* demangled = abi::__cxa_demangle(name.c_str(), NULL, NULL, &status);
    if(!status)
    {
        out << demangled << std::endl;
        free(demangled);
    }
    else
    {
        out << name << std::endl;
    }
}


template<class T, typename D>
void stream_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    // Start streaming in task thread
    ((DataEncoder<T,D>*)task)->out = &out;
    ((DataEncoder<T,D>*)task)->run_stream = true;

    // Monitor user input in this thread for stop conditions
    while(((DataEncoder<T,D>*)task)->run_stream)
    {
        std::string input;
        in >> input;
        if(input.substr(input.size()-4,4) == "stop")
        {
            ((DataEncoder<T,D>*)task)->run_stream = false;
        }
    }
    task->info("Streaming has been stopped, waiting for task to finish.");

    // Ensure any in-progress data output completes before returning
    while(((DataEncoder<T,D>*)task)->streaming)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    task->info("Streaming has ended.");
}   