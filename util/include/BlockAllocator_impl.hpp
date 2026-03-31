#pragma once

#include <memory>


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
    void* block = parent->allocate(n);        
    size_t space = parent->block_size;
    void* loc = std::align(
        alignof(T),
        sizeof(T),
        block,
        space
    );
    if(loc == nullptr) throw std::bad_alloc(); // Alignment failed
    return (T*)loc;
}


template<typename T>
void BlockAllocator<T>::deallocate(T* p, size_t n)
{
    return parent->deallocate((char*)p, n);
}
    
