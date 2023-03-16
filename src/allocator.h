#pragma once

namespace vtrz
{

class Allocator
{
public:
    virtual ~Allocator() = default;

    virtual void *allocate(size_t byteCount) = 0;
    virtual void free(void *bytes) = 0;

    template <typename T, typename... Args>
    T *allocateTyped(Args &&... args) {
        void *bytes = allocate(sizeof(T));
        T *obj = reinterpret_cast<T *>(bytes);
        new (obj) T(std::forward<Args>(args)...);
        return obj;
    }

    template <typename T, typename... Args>
    T *allocateArray(size_t count, Args &&... args) {
        void *bytes = allocate(sizeof(T) * count);
        T *objs = reinterpret_cast<T *>(bytes);
        for (size_t i = 0; i < count; ++i) {
            new (objs[i]) T(std::forward<Args>(args)...);
        }
        return objs;
    }

    template <typename T>
    void freeTyped(T *obj) {
        obj->~T();
        free(obj);
    }

    template <typename T>
    void freeArray(T *objs, size_t count) {
        for (size_t i = 0; i < count; ++i) {
            objs[i].~T();
        }
        free(objs);
    }
};

}