#pragma once

#include "Task.hpp"
#include "DataSource.hpp"
#include <type_traits>
#include <atomic>


template<class T, typename D>
void print_datatype(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T, typename D>
void stream_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);


/** Encodes arrays to transmit over console output.

Wraps a task class that must provide access to a
data stream via a method that returns a pointer
to the data to send.

@tparam T Task type that is generating the data to send.
@tparam D Underlying datatype of the data array
*/
template<class T, typename D>
class DataEncoder : public Task
{
    friend void stream_data<T,D>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

public:
    /** Create a new encoder.
    
    The given task must implement a method that 
    returns a pointer to the data, of type D.

    @param name Name of the created task
    @param task Task that will produce the data to send
    */
    DataEncoder(const char* name, std::shared_ptr<T> task) : 
        Task(name, {task}),
        task(task)
    {
        static_assert(std::is_base_of_v<DataSource<D>, T>, "Source must implement DataSource<D>");
        static_assert(std::is_base_of_v<Task, T>, "Source must be a Task type");

        declare_cli_command("datatype", &print_datatype<T,D>);
        declare_cli_command("stream", &stream_data<T,D>);
    }

    bool start_impl();

    void stop_impl();

    void step();

protected:
    std::shared_ptr<T> task;
    std::ostream* out;
    std::atomic<bool> run_stream, streaming;
    size_t size;
    D* buffer;
};


#include "DataEncoder_impl.hpp"