#pragma once

#include "assertion.h"
#include "allocator.h"
#include <algorithm>

namespace vtrz
{

template <typename T>
class DynamicArray final
{
public:
    explicit DynamicArray(Allocator *allocator) :
        _data(nullptr), _size(0), _capacity(0), _allocator(allocator) { }

    DynamicArray(Allocator *allocator, uint32_t size) :
        _allocator(allocator), _size(size), _capacity(size) {
        if (size > 0) {
            _data = allocator->allocateArray<T>(size);
        }
    }

    ~DynamicArray() {
        _allocator->freeArray<T>(_data, _size);
        _size = _capacity = 0;
        _data = nullptr;
        _allocator = nullptr;
    }

    DynamicArray(const DynamicArray &other) :
        _allocator(other._allocator) {
        copy(other);
    }

    DynamicArray(DynamicArray &&other) :
        _allocator(other._allocator) {
        swap(other);
    }

    DynamicArray &operator=(const DynamicArray &other) {
        copy(other);
        return *this;
    }

    DynamicArray &operator=(DynamicArray &&other) {
        swap(other);
        return *this;
    }

    void reserve(uint32_t newCapacity) {
        if (newCapacity >= _capacity) {
            realloc(newCapacity);
        }
    }

    void resize(uint32_t newSize) {
        if (_size > newSize) {
            for (uint32_t i = newSize; i < _size; ++i) _data[i].~T();
            _size = newSize;
        } else if (_size < newSize) {
            uint32_t oldSize = _size;
            reserve(newSize);
            _size = newSize;
            for (uint32_t i = oldSize; i < _size; ++i) new (&_data[i]) T;
        }
    }

    void push_back(const T &element) {
        if (_size >= _capacity) {
            realloc(std::max(2 * _capacity, kDefaultCapacity));
        }
        new (&_data[_size++]) T(element);
    }

    void push_back(T &&element) {
        if (_size >= _capacity) {
            realloc(std::max(2 * _capacity, kDefaultCapacity));
        }
        new (&_data[_size++]) T(std::move(element));
    }

    void pop_back() {
        ASSERT(!empty());
        _data[_size - 1].~T();
        --_size;
    }

    void clear() {
        for (uint32_t i = 0; i < _size; ++i) _data[i].~T();
        _size = 0;
    }

    uint32_t size() const { return _size; }

    bool empty() const { return _size == 0; }

    T &operator[](uint32_t index) {
        ASSERT(index < _size);
        return _data[index];
    }

    const T &operator[](uint32_t index) const {
        ASSERT(index < _size);
        return _data[index];
    }

    T *data() { return _data; }
    const T *data() const { return _data; }

    Allocator *allocator() { return _allocator; }
    const Allocator *allocator() const { return _allocator; }

    const T &front() const { ASSERT(!empty()); return _data[0]; }
    T &front() { ASSERT(!empty()); return _data[0]; }

    const T &back() const { ASSERT(!empty()); return _data[_size - 1]; }
    T &back() { ASSERT(!empty()); return _data[_size - 1]; }

    T *begin() { return _data; }
    const T *begin() const { return _data; }
    T *end() { return _data + _size; }
    const T *end() const { return _data + _size; }

    bool operator==(const DynamicArray &other) const {
        if (_size != other._size) {
            return false;
        }
        for (uint32_t i = 0; i < _size; ++i) {
            if (_data[i] != other._data[i]) {
                return false;
            }
        }
        return true;
    }

    bool operator!=(const DynamicArray &other) const {
        return !(*this == other);
    }

    void swap(DynamicArray &other) {
        if (this == &other) {
            return;
        }
        using std::swap;
        swap(_data, other._data);
        swap(_capacity, other._capacity);
        swap(_size, other._size);
        swap(_allocator, other._allocator);
    }

private:
    void copy(const DynamicArray &other) {
        if (this == &other) {
            return;
        }
        const uint32_t n = other._size;
        if (n <= _size) {
            // Overwrite the first n elements.
            std::copy(other._data, other._data + n, _data);
            // Remove the rest.
            for (uint32_t i = n; i < _size; ++i) _data[i].~T();
        } else {
            _allocator->free(_data);
            if (n > 0) {
                _data = (T *)_allocator->allocate(n * sizeof(T));
            }
            _capacity = n;
            for (uint32_t i = 0; i < n; ++i) _data[i] = other._data[i];
        }
        _size = n;
    }

    void realloc(uint32_t newCapacity)
    {
        ASSERT(newCapacity >= _size);
        if (newCapacity > 0) {
            T *newData = nullptr;
            newData = (T *)_allocator->allocate(sizeof(T) * newCapacity);
            for (uint32_t i = 0; i < _size; ++i) {
                newData[i] = std::move(_data[i]);
                _data[i].~T();
            }
            _allocator->free(_data);
            _data = newData;
            _capacity = newCapacity;
        }
    }

private:
    T *_data = nullptr;
    uint32_t _capacity = 0;
    uint32_t _size = 0;
    Allocator *_allocator = nullptr;

    static constexpr uint32_t kDefaultCapacity = 8;
};

}