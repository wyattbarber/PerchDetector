#pragma once


template<typename T>
T* DataSource<T>::acquire()
{
    std::lock_guard<std::mutex> l(mtx); // Ensure counters and locked data are updated atomically
    if(_use_counter == 0) _latest_dataset = lock(); // No data acquired yet
    _use_counter++;
    return _latest_dataset;
}


template<typename T>
void DataSource<T>::release()
{
    std::lock_guard<std::mutex> l(mtx); // Ensure counters and locked data are updated atomically
    if(_use_counter>0) _use_counter--; // proctect against multiple calls
    if(_use_counter == 0) unlock(); // Data not needed anymore
}
