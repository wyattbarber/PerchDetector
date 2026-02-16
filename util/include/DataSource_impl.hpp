#pragma once


template<typename T>
std::shared_ptr<datavalue_t<T>> DataSource<T>::acquire()
{
    std::lock_guard<std::mutex> l(mtx); 
    return latest_data;
}


template<typename T>
void DataSource<T>::swap_data(const T& value)
{
    std::lock_guard<std::mutex> l(mtx);
    if(latest_data) latest_data->stale = true;
    if constexpr (std::is_trivially_copy_constructible_v<T>)
    {
        latest_data = std::make_shared<datavalue_t<T>>(value, false);
    }
    else
    { // Must be trivially default constructible if not copy constructible
        latest_data = std::make_shared<datavalue_t<T>>();
        latest_data->stale = false;
        memcpy((void*)&latest_data->data, (void*)&value, sizeof(T));
    }
}
