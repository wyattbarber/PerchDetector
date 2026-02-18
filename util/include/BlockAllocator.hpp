#pragma once


#include <new>
#include "Logging.hpp"
#include <mutex>


/** Block allocator class.
 * 
 * Implements fixed-size allocations from a memory pool that
 * is allocated from the heap when the allocator is constructed.
 */
template<typename T>
class BlockAllocator
{
    template<typename U>
    friend class BlockAllocator;
    template<typename U>
    friend bool operator==(const BlockAllocator<T>& a, const BlockAllocator<U>& b) { return a.arena == b.arena; }
    template<typename U>
    friend bool operator!=(const BlockAllocator<T>& a, const BlockAllocator<U>& b) { return a.arena != b.arena; }

    public:
    using value_type = T;
    static constexpr size_t ctrl_block_size = 32+sizeof(BlockAllocator<T>); // Extra bytes allocated for smart pointer control blocks

    /** Create a new allocator and pool.
     * 
     * Initializes a new memory pool for up to N
     * objects of type T. The memory pool is created with malloc.
     * 
     * @param N Number of blocks to allocate.
     */
    BlockAllocator(size_t N) :
        block_size(sizeof(T) + ctrl_block_size + sizeof(size_t)),
        arena{(char*)malloc(N * block_size)},
        arena_size(N * block_size),
        next_free_block((N-1) * block_size),
        pool_id(pool_count),
        mtx(new std::mutex())
    {
        ++pool_count;
        // Initialize the first byte of each block with the index of the next free block
        for(size_t i = block_size; i < arena_size; i += block_size)
        {
            *reinterpret_cast<size_t*>(arena + i) = i - block_size;
        }
        // Block 0 is set to the last block, with a next block index exceeding the pool size
        reinterpret_cast<size_t*>(arena)[0] = arena_size+1;
    }
    ~BlockAllocator()
    {
        free(arena);
        delete mtx;
    }

    /** Rebind from another allocator type.
     * 
     * Construct an allocator that allocates from
     * another block pool.
     * 
     * @tparam U Block type of the allocator
     * 
     * @param alloc Allocator whos memory pool will be pointed to
     */
    template<typename U>
    BlockAllocator(const BlockAllocator<U>& other) :
        block_size(other.block_size),
        arena(other.arena),
        arena_size(other.arena_size),
        next_free_block(other.next_free_block),
        pool_id(other.pool_id),
        mtx(other.mtx)
    {
        static_assert(
            sizeof(U)+ctrl_block_size >= sizeof(T),
            "Cannot rebind allocator to a larger block size"
        );
    }

    /** Get a free block
     * 
     * Get the next free block in the pool.
     * 
     * If there is no free block, or if the requested
     * number of blocks is not 1, std::bad_alloc 
     * is thrown.
     */
    T* allocate(size_t n)
    {
        std::lock_guard<std::mutex> l(*mtx);
        if(n != 1) throw std::bad_alloc();
        if(next_free_block > arena_size) throw std::bad_alloc();        
        auto next = *reinterpret_cast<size_t*>(arena + next_free_block);
        char* out = arena + next_free_block + sizeof(size_t);
        report(next_free_block / block_size, (void*)out, true);
        next_free_block = next;
        return (T*)out;
    }

    /** Frees the given block.
     * 
     * Since only single blocks may be allocated, 
     * n is ignored.
     */
    void deallocate(T* p, size_t n)
    {
        std::lock_guard<std::mutex> l(*mtx);
        if(next_free_block > arena_size)
        {
            // This block is now the last in the list
            *reinterpret_cast<size_t*>(arena) = arena_size + 1;
        }
        else
        {
            *reinterpret_cast<size_t*>((char*)p - sizeof(size_t)) = next_free_block;
        }
        next_free_block = ((char*)p - arena) - sizeof(size_t);
        report(next_free_block / block_size, (void*)p, false);
    }

    protected:
    const size_t block_size; // Total block size for object, control block, and free list index 
    char * const arena; // Memory pool
    const size_t arena_size; // Total available bytes
    size_t next_free_block; // Index of next available block
    const unsigned pool_id;
    static unsigned pool_count;
    std::mutex * const mtx;

    void report(size_t block_id, void* addr, bool alloc)
    {
        if(alloc) Logger::instance() << "[INFO][_BlockAllocator] Allocated " << typeid(T).name() << " in block " << block_id << " (" << addr << ") in pool " << pool_id << std::endl;
        else Logger::instance() << "[INFO][_BlockAllocator] Freed " << typeid(T).name() << " from block " << block_id << " (" << addr << ") in pool " << pool_id << std::endl;;
    }
};

template<typename T>
unsigned BlockAllocator<T>::pool_count = 0;
