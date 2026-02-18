#pragma once


template<class T>
std::shared_ptr<typename DataSource<T>::update_type> DataSource<T>::acquire()
{
    std::lock_guard<std::mutex> l(mtx); 
    return latest_data;
}


template<class T>
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


template<class T>
void DataSource<T>::swap_data(const DataSource<T>::value_type& value)
{
    std::lock_guard<std::mutex> l(mtx);
    if(latest_data) latest_data->stale = true;
    if constexpr (std::is_trivially_copy_constructible_v<T>)
    {
        latest_data = std::make_shared<update_type>(value, false);
    }
    else
    { // Must be trivially default constructible if not copy constructible
        latest_data = std::make_shared<update_type>();
        latest_data->stale = false;
        memcpy((void*)&latest_data->data, (void*)&value, sizeof(T));
    }
}
