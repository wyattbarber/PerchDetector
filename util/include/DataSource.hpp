#pragma once

#include "Task.hpp"
#include <mutex>


/** Datatype for data value updates with metadata.
 * 
 * Contains a datapoint from a datasource, 
 * along with a stale flag that is set true once the 
 * source of the data has produced a new update.
 * 
 * Intended to only ever be created as a shared pointer
 * by the DataSource object that creates the data it contains.
 * 
 * @tparam T Type of the wrapped datapoint
 */
template<typename T>
struct _datavalue_t 
{
    T data; /// This updates data value
    bool stale; /// True if the data source has a newer update
};
template<typename T>
using datavalue_t = _datavalue_t<T>;


/** Interface class definition for tasks that provide thread-safe data.

Assumes that all data sources only need to provide the most recent
dataset (as opposed to maintaining a queue of all measurments).

Wraps updates in a struct that contains a flag for data consumers
to check if the data they have is stale, and ensures that data updates
are managed by smart pointers.

The datatype of the updates must be trivially default or copy constructible
to work with this interface.

@tparam T Type of the data provided.
*/
template<typename T>
class DataSource : public Task
{
public:
    DataSource(const char* name, std::initializer_list<std::shared_ptr<Task>> dependencies) : 
        Task(name, dependencies), 
        latest_data() 
    { 
        static_assert(
            std::is_trivially_copy_constructible_v<T> || std::is_trivially_default_constructible_v<T>, 
            "Datatypes of the DataSource interface must be trivially copy or default constructible."
        ); 
    }

    /** Provides access to the latest data value.
     * 
     * Creates a shared pointer to a datavalue_t type
     * containing the latest data value and its metadata.
     *     
    @return Pointer to latest data value
    */
    std::shared_ptr<datavalue_t<T>> acquire();

    /** Starts the datasource.
     * 
     * Overrides the base Task implementation to ensure
     * at least one datavalue is available before dependent tasks
     * start.
     *  */    
    bool start() override;

protected:
    
    /** Provide a new data value.
     * 
     * Will creae a copy of the passed data value wrapped
     * in a datavalue_t instance, and updates the shared_ptr
     * that gets returned by acquire().
     * 
     * If the type T is copy constructible, then the copy will be
     * performed with the copy constructor. Otherwise the copy 
     * will be done with memcpy().
     * 
     * Will set the stale flag of the current data value before 
     * swapping it, to notify consumers that new data is available.
     * 
     * @param value New data point to update
     */
    void swap_data(const T& value);

private:
    std::shared_ptr<datavalue_t<T>> latest_data;
    std::mutex mtx;
};


#include "DataSource_impl.hpp"