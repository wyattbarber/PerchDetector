#pragma once

#include "DataSource.hpp"
#include <type_traits>
#include <atomic>
#include <unistd.h>

template<class T, class C>
void data_dims(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T, class C>
void map_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<class T, class C>
void unmap_data(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

template<typename C>
void print_mapper_datatype(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);


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
template<class T, typename C>
class DataMapper : public TaskBase<DataMapper<T,C>>
{
    friend void map_data<T,C>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);
    friend void unmap_data<T,C>(Task* task, std::istream& in, std::ostream& out, const std::vector<std::string>& args);

public:
    /** Create a new data mapper.
    
    @param name Name of the created task
    @param task Task that will produce the data to send
    @param converter Optional converter function to apply to the data
    */
    DataMapper(const char* name, std::shared_ptr<DataSource<T>> task, C converter) : 
        TaskBase<DataMapper>(name, {task}),
        task(task),
        converter(converter),
        running(false),
        mapping(false)
    {
        this->declare_cli_command("datatype", &print_mapper_datatype<typename C::conversion_type>);
        this->declare_cli_command("dimensions", &data_dims<T,C>);
        this->declare_cli_command("map", &map_data<T,C>);
        this->declare_cli_command("unmap", &unmap_data<T,C>);
    }

    bool start_impl();

    void stop_impl();

    void step();

    std::vector<size_t> dims(){ return converter.dims(); }

protected:
    std::shared_ptr<DataSource<T>> task;
    typename T::update_ptr_const_type latest;
    const C converter;
    std::FILE* map_file;
    void* map;
    std::atomic<bool> running, mapping;
};


/** Default no-op data converter.

*/
template<typename T>
struct no_op_converter
{ 
    no_op_converter(std::shared_ptr<T> task) : task(task) {}
    std::shared_ptr<T> task;

    using conversion_type = typename T::value_type;
    std::vector<size_t> dims() const { return task->dims(); }
    static void eval(void* dst, const conversion_type& src) { memcpy(dst, (const void*)&src, sizeof(conversion_type)); } 
};


/** Helper to create a new data mapper.

Constructs a new shared pointer to a data mapper instance
with the given conversion functor.

The converter argument is optional, it can be used to reformat data or isolate 
specific components before sharing it to other processes. If omitted, then data will be 
copied to the memory map region as-is.

@tparam T Type of the task producing data.
@tparam C Type of the conversion functor to apply.

@param name Name of the DataMapper task to create.
@param task Task producing the data to map.
@param converter Converter functor to apply to data updates.

@return new task type.
*/
template<class T, typename C>
auto make_data_mapper(const char* name, std::shared_ptr<T> task, C converter)
{
    return std::make_shared<DataMapper<T,C>>(name, task, converter);
}
template<class T>
auto make_data_mapper(const char* name, std::shared_ptr<T> task)
{
    return std::make_shared<DataMapper<T,no_op_converter<T>>>(name, task, no_op_converter<T>(task));
}

#include "DataMapper_impl.hpp"