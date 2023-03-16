#pragma once

#include "blockAllocator.h"
#include "util.h"
#include <algorithm>

namespace vtrz
{

BlockAllocator::BlockAllocator(size_t defaultBlockSize) : defaultBlockSize(defaultBlockSize) {}

BlockAllocator::~BlockAllocator()
{
    freeAligned(currBlock);
    for (auto &block : usedBlocks) {
        freeAligned(block.second);
    }
    for (auto &block : freeBlocks) {
        freeAligned(block.second);
    }
}

void *BlockAllocator::allocate(size_t byteCount)
{
    constexpr size_t alignment = alignof(std::max_align_t);

    byteCount = (byteCount + alignment - 1) & ~(alignment - 1);
    if (currBlockPos + byteCount > currAllocSize) {
        // Add current block to usedBlocks list.
        if (currBlock) {
            usedBlocks.push_back(std::make_pair(currAllocSize, currBlock));
            currBlock = nullptr;
            currAllocSize = 0;
        }

        // Get a new block of memory.

        // Try to get memory block from freeBlocks.
        for (size_t i = 0; i < freeBlocks.size(); ++i) {
            if (freeBlocks[i].first >= byteCount) {
                currAllocSize = freeBlocks[i].first;
                currBlock = freeBlocks[i].second;
                // (swap and shrink)
                if (i + 1 < freeBlocks.size()) {
                    std::swap(freeBlocks[i], freeBlocks.back());
                }
                freeBlocks.resize(freeBlocks.size() - 1);
                break;
            }
        }
        // Didn't find one free. Need to allocate a new block.
        if (!currBlock) {
            currAllocSize = std::max(byteCount, defaultBlockSize);
            constexpr size_t kCacheLine = 64;
            currBlock = (byteptr_t)allocAligned(currAllocSize, kCacheLine);
        }
        currBlockPos = 0;
    }
    void *ret = currBlock + currBlockPos;
    currBlockPos += byteCount;
    return ret;
}

void BlockAllocator::free(void *bytes)
{
    (void)bytes;
    // This function is a no-op.
    // PoolAllocator can only grow. It can only free all allocated memory at once (during destruction).
}

void BlockAllocator::reset()
{
    freeBlocks.insert(freeBlocks.end(), std::make_move_iterator(usedBlocks.begin()), std::make_move_iterator(usedBlocks.end()));
    usedBlocks.clear();
}

}