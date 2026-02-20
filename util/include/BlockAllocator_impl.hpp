#pragma once


template<typename T>
template<typename U>
BlockAllocator<T>::BlockAllocator(const BlockAllocator<U>& other) : parent(other.parent)
{
    static_assert(
        sizeof(T) <= sizeof(U)+(BlockPool::ctrl_block_size),
        "Cannot rebind allocator to a larger block size"
    );
}


template<typename T>
T* BlockAllocator<T>::allocate(size_t n)
{
    return (T*)parent->allocate(n);        
}

template<typename T>
void BlockAllocator<T>::deallocate(T* p, size_t n)
{
    return parent->deallocate((char*)p, n);
}
    
