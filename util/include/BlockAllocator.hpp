#pragma once


#include <new>
#include "Logging.hpp"
#include <mutex>

class BlockPool;

/** Block allocator
 * 
 * Acts as an interface between the BlockPool and standard library functions
 * like allocate_shared.
 * 
 * Intended to be lighter-weight for copying. Manages a single memory
 * block, and should be created by the BlockPool instance that is its parent.
 * 
 * @tparam T Datatype the block is made to store
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

    BlockAllocator(BlockPool* parent) : parent(parent)
    {}

    /** Rebind from another allocator type.
     * 
     * Construct an allocator that allocates from
     * another block pool.
     * 
     * @tparam U Block type of the allocator
     * 
     * @param other Allocator whos memory pool will be pointed to
     */
    template<typename U>
    BlockAllocator(const BlockAllocator<U>& other);


    T* allocate(size_t n);

    /** Frees the given block.
     * 
     * Since only single blocks may be allocated, 
     * n is ignored.
     */
    void deallocate(T* p, size_t n);
    
    protected:
        BlockPool * const parent;
};


/** Block pool manager class.
 * 
 * Implements fixed-size allocations from a memory pool that
 * is allocated from the heap when the allocator is constructed.
 */
class BlockPool
{
    template<typename T>
    friend class BlockAllocator;

    public:
    static constexpr size_t ctrl_block_size = 32+sizeof(BlockAllocator<char>); // Extra bytes allocated for container control blocks
    const size_t block_size; // Total block size for object, control block, and free list index 

    /** Create a new allocator and pool.
     * 
     * Initializes a new memory pool for up to N
     * objects of type T. The memory pool is created with malloc.
     * 
     * @param N Number of blocks to allocate
     * @param S Size of each block
     */
    BlockPool(size_t N, size_t S) :
        block_size(S + ctrl_block_size + sizeof(size_t) + alignof(size_t)),
        arena((char*)std::aligned_alloc(alignof(std::max_align_t), N * block_size)),
        arena_size(N * block_size),
        next_free_block(N-1)
#ifdef LOG_ALLOC
        , pool_id(pool_count++)
#endif
    {
        // Initialize the first byte of each block with the index of the next free block
        for(size_t i = 1; i < N; i++)
        {
            *next_free_from_block_id(i) = i-1;
        }
        // Block 0 is set to the last block, with a next block index exceeding the pool size
        *next_free_from_block_id(0) = arena_size+1;

#ifdef LOG_ALLOC
        Logger::instance() << "[INFO][_BlockPool_" << pool_id << "] Initialized with " << N << " blocks of " << block_size << " bytes at " << (void*)arena << std::endl;
        Logger::instance() << "[INFO][_BlockPool_" << pool_id << "] Memory span is " << (void*)arena << " to " << (void*)(arena + arena_size) << std::endl;
#endif
    }
    
    ~BlockPool()
    {
        delete[] arena;
#ifdef LOG_ALLOC
        Logger::instance() << "[INFO][_BlockPool_" << pool_id << "] Deleted." << std::endl;
#endif
    }
    
    template<typename T>
    BlockAllocator<T> make_allocator(){ return BlockAllocator<T>(this); }

    protected:
    /** Get a free block
     * 
     * Get the next free block in the pool.
     * 
     * If there is no free block, or if the requested
     * number of blocks is not 1, std::bad_alloc 
     * is thrown.
     */
    char* allocate(size_t n);

    /** Frees the given block.
     * 
     * Since only single blocks may be allocated, 
     * n is ignored.
     */
    void deallocate(char* p, size_t n);

    protected:
    char * const arena; // Memory pool
    const size_t arena_size; // Total available bytes
    size_t next_free_block; // Block ID of next available block
    std::mutex mtx;
#ifdef LOG_ALLOC
    const unsigned pool_id;
    static unsigned pool_count;

    void report(size_t block_id, void* addr, bool alloc);
#endif

    size_t block_id_from_ptr(char* ptr);

    char* data_ptr_from_block_id(size_t id);

    size_t* next_free_from_block_id(size_t id);
};


#include "BlockAllocator_impl.hpp"