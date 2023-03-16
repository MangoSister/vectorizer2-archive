#pragma once

#include "assertion.h"
#include "maths.h"
#include <cstdint>
#include <cstring>
#include <functional>
#include <type_traits>

namespace vtrz
{

inline void *allocAligned(size_t size, size_t alignment)
{
#if defined(_MSC_VER)
    return _aligned_malloc(size, alignment);
#elif defined(__OSX__)
    /* OSX malloc already returns 16-byte aligned data suitable
       for AltiVec and SSE computations */
    return malloc(size);
#else
    return memalign(alignment, size);
#endif
}

inline void freeAligned(void *ptr)
{
#if defined(_MSC_VER)
    _aligned_free(ptr);
#else
    free(ptr);
#endif
}

enum class Alignment : uint8_t
{
    natural,
    force,
};

// Variable length array on stack. Be very careful about this!!
#define VLA(PTR, TYPE, COUNT) \
static_assert(std::is_trivially_destructible<TYPE>::value, "Don't use VLA for non trivially destructible types.");  \
TYPE *PTR = (TYPE *)alloca((COUNT) * sizeof(TYPE));                                                                 \
{ for (uint32_t __i = 0; __i < COUNT; ++__i) new (PTR + __i) TYPE(); }                                              \

template <typename T>
struct PreAllocVector
{
    PreAllocVector(T *data, uint32_t capacity) : data(data), capacity(capacity), usedCount(0) { }

    void clear() {
        usedCount = 0;
    }

    void push_back(const T &value) {
        ASSERT_FATAL(usedCount <= capacity);
        data[usedCount++] = value;
    }

    void push_back(T &&value) {
        ASSERT_FATAL(usedCount <= capacity);
        data[usedCount++] = std::move(value);
    }

    T *begin() { return data; }
    const T *begin() const { return data; }
    T *end() { return data + usedCount; }
    const T *end() const { return data + usedCount; }

    const T &operator[](uint32_t index) const {
        ASSERT(index < usedCount);
        return data[index];
    }

    T &operator[](uint32_t index) {
        ASSERT(index < usedCount);
        return data[index];
    }

    uint32_t size() const {
        return usedCount;
    }

    bool empty() const {
        return usedCount == 0;
    }

    const T &front() const {
        ASSERT(usedCount > 0);
        return data[0];
    }

    T &front() {
        ASSERT(usedCount > 0);
        return data[0];
    }

    const T &back() const {
        ASSERT(usedCount > 0);
        return data[usedCount - 1];
    }

    T &back() {
        ASSERT(usedCount > 0);
        return data[usedCount - 1];
    }

    T *data = nullptr;
    uint32_t usedCount = 0;
    uint32_t capacity = 0;
};

template <typename T, Alignment TAlign = Alignment::natural>
struct StaticArray
{
    template <Alignment U = TAlign>
    typename std::enable_if<U == Alignment::force>::type
        construct(uint32_t size) {
        data = (T *)allocAligned(size * sizeof(T), alignof(T));
        for (uint32_t i = 0; i < size; ++i) {
            new (data + i) T();
        }
    }

    template <Alignment U = TAlign>
    typename std::enable_if<U == Alignment::natural>::type
        construct(uint32_t size) {
        data = new T[size];
    }

    template <Alignment U = TAlign>
    typename std::enable_if<U == Alignment::force>::type
        destroy() {
        for (uint32_t i = 0; i < size; ++i) {
            data[i].~T();
        }
        freeAligned(data);
        data = nullptr;
    }

    template <Alignment U = TAlign>
    typename std::enable_if<U == Alignment::natural>::type
        destroy() {
        delete[] data;
        data = nullptr;
    }

    ~StaticArray() {
        destroy();
        size = 0;
    }

    StaticArray() : size(0), data(nullptr) {}

    explicit StaticArray(uint32_t size) : size(size) {
        construct(size);
    }

    StaticArray(const StaticArray &other) {
        size = other.size;
        construct(size);
        std::copy(other.begin(), other.end(), begin());
    }

    StaticArray(StaticArray &&other) {
        size = other.size;
        data = other.data;
        other.size = 0;
        other.data = nullptr;
    }

    StaticArray &operator=(const StaticArray &other) {
        if (this == &other) {
            return *this;
        }
        destroy();
        size = other.size;
        construct(size);
        std::copy(other.begin(), other.end(), begin());
        return *this;
    }

    StaticArray &operator=(StaticArray &&other) {
        if (this == &other) {
            return *this;
        }
        std::swap(size, other.size);
        std::swap(data, other.data);
        return *this;
    }

    const T &operator[](uint32_t index) const {
        ASSERT(index < size);
        return data[index];
    }

    T &operator[](uint32_t index) {
        ASSERT(index < size);
        return data[index];
    }

    T *begin() { return data; }
    const T *begin() const { return data; }
    T *end() { return data + size; }
    const T *end() const { return data + size; }

    bool empty() const { return data == nullptr; }

    T *data = nullptr;
    uint32_t size = 0;
};

template<typename T, uint32_t N>
constexpr uint32_t countOf(T(&)[N]) {
    return N;
}

template <typename K, typename V, uint32_t N, typename Comp = std::less<K>>
struct HeapArray
{
    HeapArray(const Comp &comp = Comp()) : comp(comp), n(0) {}

    const K &minKey() const { return keys[0]; }
    const V &minValue() const { return values[0]; }
    uint32_t size() const { return n; }
    bool empty() const { return n == 0; }
    void clear() { n = 0; }

    void push(const K &key, const V &value) {
        if (n == N) return;

        keys[n] = key;
        values[n] = value;
        ++n;

        uint32_t k = n;
        while (k > 0) {
            uint32_t p = (k - 1) / 2;
            if (!comp(keys[p], keys[k])) break;

            std::swap(keys[p], keys[k]);
            std::swap(values[p], values[k]);
            k = p;
        }
    }

    void pop() {
        if (n == 0) return;

        std::swap(keys[n - 1], keys[0]);
        std::swap(values[n - 1], values[0]);
        --n;

        uint32_t k = 0;
        while (2 * k + 1 < n) {
            uint32_t j = 2 * k + 1;
            if (j < n && comp(keys[j], keys[j + 1])) ++j;
            if (!comp(keys[k], keys[j])) break;

            std::swap(keys[k], keys[j]);
            std::swap(values[k], values[j]);
            k = j;
        }
    }

    K keys[N];
    V values[N];
    Comp comp;
    uint32_t n = 0;
};

constexpr uint64_t bitMix64(uint64_t x)
{
    // Constants taken from (mix 13) http://zimbry.blogspot.com/2011/09/better-bit-mixing-improving-on.html
    x = (x ^ (x >> 30)) *UINT64_C(0xbf58476d1ce4e5b9);
    x = (x ^ (x >> 27)) *UINT64_C(0x94d049bb133111eb);
    x = x ^ (x >> 31);
    return x;
}

constexpr size_t hashCombine(size_t lhs, size_t rhs)
{
    lhs ^= rhs + UINT64_C(0x9e3779b97f4a7c17) + (lhs << 6) + (lhs >> 2); // For 32-bit size_t, use 0x9e3779b9
    return lhs;
}

}