#pragma once


#include "DataSource.hpp"
#include <type_traits>
#include <atomic>


template<class T>
void print_datatype(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T>
void stream_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);


/** Encodes arrays to transmit over console output.

Wraps a task class that must provide access to a
data stream via a method that returns a pointer
to the data to send.

@tparam T Task type that is generating the data to send.
*/
template<class T>
class DataEncoder : public TaskBase<DataEncoder<T>>
{
    friend void stream_data<T>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

public:
    /** Create a new encoder.
    
    The given task must implement a method that 
    returns a pointer to the data, of type D.

    @param name Name of the created task
    @param task Task that will produce the data to send
    */
    DataEncoder(const char* name, std::shared_ptr<DataSource<T>> task) : 
        TaskBase<DataEncoder<T>>(name, {task}),
        task(task),
        run_stream(false),
        streaming(false)
    {
        declare_cli_command("datatype", &print_datatype<T>);
        declare_cli_command("stream", &stream_data<T>);
    }

    bool start_impl();

    void stop_impl(){}

    void step();

protected:
    std::shared_ptr<DataSource<T>> task;
    std::ostream* out;
    std::atomic<bool> run_stream, streaming;
};


#include "DataEncoder_impl.hpp"