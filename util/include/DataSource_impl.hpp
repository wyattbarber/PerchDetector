#pragma once


template<typename T>
std::shared_ptr<datavalue_t<T>> DataSource<T>::acquire()
{
    std::lock_guard<std::mutex> l(mtx); 
    return latest_data;
}


template<typename T>
bool DataSource<T>::start()
{
    auto res = Task::start();
    while(!acquire())
    { 
        info("Waiting for first data to be produced...");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    info("Initial data pointer is valid.");
    return res;
}


template<typename T>
void DataSource<T>::swap_data(const T& value)
{
    std::lock_guard<std::mutex> l(mtx);
    if(latest_data) latest_data->stale = true;
    if constexpr (std::is_trivially_copy_constructible_v<T>)
    {
        latest_data = std::allocate_shared<datavalue_t<T>>(pool, value, false);
    }
    else
    { // Must be trivially default constructible if not copy constructible
        latest_data = std::allocate_shared<datavalue_t<T>>(pool);
        latest_data->stale = false;
        memcpy((void*)&latest_data->data, (void*)&value, sizeof(T));
    }
}
