#pragma once

#include "Task.hpp"
#include "BlockAllocator.hpp"
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


/** Forward-declared static interface specs for datasource tasks.
 * 
 * Must define a type "value_type", which is the datatype 
 * provided to the swap_data() method of DataSource<T>.
 * 
 * @tparam T Task type
 */
template<class T>
struct DataSourceTraits {};


/** Utility macro to create forward declarations for a datasource class.
 * 
 * @param C Class name being declared
 * @param T Datatype produced by the declared class
 */
#define FWD_DECL_DATA_SOURCE(C, T) \
class C; \
 \
template<> \
struct DataSourceTraits<C> \
{ \
    using value_type = T; \
}; 

/** Interface class definition for tasks that provide thread-safe data.

Assumes that all data sources only need to provide the most recent
dataset (as opposed to maintaining a queue of all measurments).

Wraps updates in a struct that contains a flag for data consumers
to check if the data they have is stale, and ensures that data updates
are managed by smart pointers.

The datatype of the updates must be trivially default or copy constructible
to work with this interface.

@tparam T Derived task type implementing this interface.
*/
template<class T>
class DataSource : public Task
{
public:
    using value_type = typename DataSourceTraits<T>::value_type;
    using update_type = _datavalue_t<value_type>;

    DataSource(const char* name, std::initializer_list<std::shared_ptr<Task>> dependencies) : 
        Task(name, dependencies), 
        latest_data()
#ifdef BLOCK_ALLOC
        , pool(10, sizeof(update_type))
#endif
    { 
        static_assert(
            std::is_trivially_copy_constructible_v<value_type> || std::is_trivially_default_constructible_v<value_type>, 
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
    std::shared_ptr<update_type> acquire();

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
    void swap_data(const value_type& value);

private:
    std::shared_ptr<update_type> latest_data;
    std::mutex mtx;
#ifdef BLOCK_ALLOC
    BlockPool pool;
#endif
};


#include "DataSource_impl.hpp"