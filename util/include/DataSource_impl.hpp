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
#ifdef BLOCK_ALLOC
    info("Configured to allocate data values from custom block pool. Datatype size: ", sizeof(value_type), ", block size: ", pool.block_size);
#endif
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
#ifdef BLOCK_ALLOC
        latest_data = std::allocate_shared<update_type>(pool.make_allocator<update_type>(), value, false);
#else
        latest_data = std::make_shared<update_type>(value, false);
#endif
    }
    else
    { // Must be trivially default constructible if not copy constructible
#ifdef BLOCK_ALLOC
        latest_data = std::allocate_shared<update_type>(pool.make_allocator<update_type>());
#else
        latest_data = std::make_shared<update_type>();
#endif
        latest_data->stale = false;
        memcpy((void*)&latest_data->data, (void*)&value, sizeof(value_type));
    }
}
