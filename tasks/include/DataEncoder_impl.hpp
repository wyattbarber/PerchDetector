#pragma once

#include <cxxabi.h>
#include <sstream>
#include <chrono>
#include <thread>


template<class T>
bool DataEncoder<T>::start_impl(){ 
    if(!task->is_alive())
    {
        error("Cannot start streamer if producer is not running.");
        return false;
    }
    return true;
}


template<class T>
void DataEncoder<T>::stop_impl()
{
    info("Freeing streaming buffer.");
}


template<class T>
void DataEncoder<T>::step()
{
    if(run_stream)
    {
        streaming = true;
        auto data = task->acquire();
        char* buffer = reinterpret_cast<char*>(&(data->data));

        // Dump bytes of all data to output
        for(size_t i = 0; i < sizeof(typename T::value_type); ++i)
        {
            if(!run_stream) break; // Check for exit mid frame
            (*out) << buffer[i];
        }
        tick();
    }
    else
    {
        streaming = false;
    }
}


template<class T>
void print_datatype(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    int status;
    std::string name = typeid(typename T::value_type).name();
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


template<class T>
void stream_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args)
{
    // Start streaming in task thread
    task->info("Starting data stream.");
    ((DataEncoder<T>*)task)->out = &out;
    ((DataEncoder<T>*)task)->run_stream = true;

    // Monitor user input in this thread for stop conditions
    while(((DataEncoder<T>*)task)->run_stream)
    {
        std::string input;
        in >> input;
        if(input.substr(input.size()-4,4) == "stop")
        {
            ((DataEncoder<T>*)task)->run_stream = false;
        }
    }
    task->info("Streaming has been stopped, waiting for task to finish.");

    // Ensure any in-progress data output completes before returning
    while(((DataEncoder<T>*)task)->streaming)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    task->info("Streaming has ended.");
}   