#pragma once

#include <mutex>

/** Interface class definition for tasks that provide thread-safe data.

Assumes that all data sources only need to provide the most recent
dataset (as opposed to maintaining a queue of all measurments).

Tracks a use counter to handle multiple consumers locking the data
simultaneously, however this relies on the consumers ensure they do
not make repeated calls that would cause the use counter to innacurately
reflect the number of consumers locking the data.

@tparam T scalar type of the data provided.
*/
template<typename T>
class DataSource
{
public:
    DataSource()
    {
        _use_counter = 0;
    }

    /** Provides access to the latest data value.
    
    Returns the current data pointer and increases
    the use counter. If the use counter is 0 when called,
    then the latest version of the dataset is aquired and locked.

    Must be paired with a call to release() once 
    the consumer is done with this data.
    
    @return Pointer to dataset
    */
    T* acquire();

    /** Release lock on the dataset
    
    Indicates that one consumer no longer needs
    the dataset. Must be paired with exactly one previous
    call to acquire().

    Decreases the use counter, and unlocks the data
    if this reduces it to 0;
    */
    void release();

    /** Get the size of the dataset.
    
    Must be implemented to provide the number
    of elements of type T available in the data.

    @return Number of elements
    */
    virtual size_t size() = 0;

protected:
    /** Must be implemented to provide access to dataset.
    
    Defines dataset specific locking and returns the pointer
    to the latest valid data.

    @return Pointer to data
    */
    virtual T* lock() = 0;

    /** Must be implemented to unlock dataset.

    Implements dataset specific unlocking to 
    indicate that the pointer previously returned
    by lock() is no longer needed and its data can 
    be changed.
    */
    virtual void unlock() = 0;

    unsigned _use_counter;
    T* _latest_dataset;
    std::mutex mtx;
};


#include "DataSource_impl.hpp"