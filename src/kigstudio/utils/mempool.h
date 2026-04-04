#pragma once
#include <atomic>
#include <mutex>
namespace sinriv::kigstudio {

template <typename T>
concept mempool_c = requires(T a) { a.next; };

template <mempool_c T>
class mempool {
   public:
    T* pool = nullptr;
    inline T* get() {
        if (pool) {
            T* r = pool;
            pool = pool->next;
            return r;
        } else {
            return new T;
        }
    }
    inline void del(T* f) {
        f->next = pool;
        pool = f;
    }
};

}  // namespace sinriv::kigstudio
