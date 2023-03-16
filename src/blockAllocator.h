#pragma once

#include "allocator.h"
#include <vector>
#include <utility>

namespace vtrz
{

class BlockAllocator final : public Allocator
{
public:
    explicit BlockAllocator(size_t defaultBlockSize = 1024);
    ~BlockAllocator();

    void *allocate(size_t byteCount) final;
    void free(void *bytes) final;

    void reset();

private:
    using byteptr_t = uint8_t *;

    size_t defaultBlockSize = 0;
    size_t currBlockPos = 0;
    size_t currAllocSize = 0;
    byteptr_t currBlock = nullptr;
    std::vector<std::pair<size_t, byteptr_t>> usedBlocks;
    std::vector<std::pair<size_t, byteptr_t>> freeBlocks;
};

}