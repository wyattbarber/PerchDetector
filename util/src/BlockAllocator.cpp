#include "BlockAllocator.hpp"


unsigned BlockPool::pool_count = 0;


char* BlockPool::allocate(size_t n)
{
    std::lock_guard<std::mutex> l(mtx);
    if(n != 1) throw std::bad_alloc();
    if(next_free_block > arena_size) throw std::bad_alloc();        

    size_t next = *next_free_from_block_id(next_free_block);
    char* out = data_ptr_from_block_id(next_free_block);
    report(next_free_block, (void*)out, true);
    next_free_block = next;
    return out;
}

   
void BlockPool::deallocate(char* p, size_t n)
{
    size_t id = block_id_from_ptr(p);
    report(id, (void*)p, false);

    std::lock_guard<std::mutex> l(mtx);
    // This block is now the last in the list, either the last or replacing the current next block
    if(next_free_block > arena_size)
    {
        // Only free block
        *next_free_from_block_id(id) = arena_size + 1;
    }
    else
    {
        // Insert at top of stack
        *next_free_from_block_id(id) = next_free_block;
    }
    // This is now thenext block to allocate
    next_free_block = id;
}


void BlockPool::report(size_t block_id, void* addr, bool alloc)
{
    if(alloc) Logger::instance() << "[INFO][_BlockPool_" << pool_id << "] Allocated block " << block_id << ", data at " << addr << std::endl;
    else Logger::instance() << "[INFO][_BlockPool_" << pool_id << "] Freeing block " << block_id << ", data at " << addr << std::endl;;
}


size_t BlockPool::block_id_from_ptr(char* ptr)
{
    std::ptrdiff_t offset = ptr - arena;
    return offset / block_size;
}


char* BlockPool::data_ptr_from_block_id(size_t id)
{
    char* base = arena + (id * block_size);
    return base + sizeof(size_t);
}


size_t* BlockPool::next_free_from_block_id(size_t id)
{        
    char* base = arena + (id * block_size);
    return reinterpret_cast<size_t*>(base);
}