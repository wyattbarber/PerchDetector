#pragma once

#include "DataSource.hpp"
#include "DataEncoder.hpp"
#include <type_traits>
#include <atomic>
#include <unistd.h>

template<class T>
void data_dims(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T>
void map_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T>
void unmap_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

/** Default no-op data converter.

*/
template<typename T>
void no_op_converter(void* dst, const T& src)
{ 
    memcpy(dst, (const void*)&src, sizeof(T)); 
}


/** Shares data with other processes

Provides an interface for the output of a DataSource to be
copied to a memory mapped file. Defines a CLI function
that will accept a file name and open it for memory mapping, 
continually updating the file as new data is available.

The memory mapped data size and type can be accessed via
additional CLI commands. The format of the memory mapped file is
as follows:

- Byte 0: Lock flag, set to non-zero when this task is writing to the data
- Bytes 1-N: Data

@tparam T Task type that is generating the data to send.
*/
template<class T, typename C = typename T::value_type, typename CF = decltype(no_op_converter<typename T::value_type>)>
class DataMapper : public Task
{
    friend void map_data<T>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);
    friend void unmap_data<T>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

public:
    /** Create a new data mapper.
    
    @param name Name of the created task
    @param task Task that will produce the data to send
    @param converter Optional converter function to apply to the data
    */
    DataMapper(const char* name, std::shared_ptr<DataSource<T>> task, CF& converter = no_op_converter<typename T::value_type>) : 
        Task(name, {task}),
        task(task),
        converter(converter),
        running(false),
        mapping(false)
    {
        declare_cli_command("datatype", &print_datatype<T>);
        declare_cli_command("dimensions", &data_dims<T>);
        declare_cli_command("map", &map_data<T>);
        declare_cli_command("unmap", &unmap_data<T>);
    }

    bool start_impl();

    void stop_impl();

    void step();

protected:
    std::shared_ptr<DataSource<T>> task;
    typename T::update_ptr_const_type latest;
    CF& converter;
    std::FILE* map_file;
    void* map;
    std::atomic<bool> running, mapping;
};


#include "DataMapper_impl.hpp"