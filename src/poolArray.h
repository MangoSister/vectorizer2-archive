#pragma once
#include <cstdint>
#include <numeric>
#include <vector>

using PoolArrayIndex = uint32_t;

template <typename T>
class PoolArray
{
public:
    explicit PoolArray(uint32_t preAllocSize = 256) {
        if (preAllocSize) {
            data.resize(preAllocSize);
            freeList.resize(preAllocSize);
            std::iota(std::rbegin(freeList), std::rend(freeList), 0);
        }
    }

    PoolArrayIndex append() {
        PoolArrayIndex index;
        if (freeList.size()) {
            index = freeList.back();
            freeList.pop_back();
        } else {
            index = (uint32_t)data.size();
            data.resize(data.size() + 1);
        }
        return index;
    }
    void remove(uint32_t index) {
        freeList.push_back(index);
    }

    T &operator[](uint32_t index) {
        return data[index];
    }
    const T &operator[](uint32_t index) const {
        return data[index];
    }

    std::vector<uint32_t> allocatedList() const {
        std::vector<bool> flags(data.size(), true);
        for (uint32_t i = 0; i < (uint32_t)freeList.size(); ++i) {
            flags[freeList[i]] = false;
        }
        std::vector<uint32_t> allocated;
        allocated.reserve(data.size() - freeList.size());
        for (uint32_t i = 0; i < (uint32_t)data.size(); ++i) {
            if (flags[i]) {
                allocated.push_back(i);
            }
        }
        return allocated;
    }

private:
    std::vector<T> data;
    std::vector<uint32_t> freeList;
};