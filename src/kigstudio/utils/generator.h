#pragma once

#include <coroutine>

namespace sinriv::kigstudio {

    template <typename T>
    class Generator {
    public:
        struct promise_type {
            inline Generator get_return_object() {
                return Generator{ handle_type::from_promise(*this) };
            }

            inline static Generator get_return_object_on_allocation_failure() {
                return Generator{ nullptr };
            }

            inline auto initial_suspend() {
                return std::suspend_always{};
            }

            inline auto final_suspend() noexcept {
                return std::suspend_always{};
            }

            inline auto yield_value(T value) {
                current_value = value;

                return std::suspend_always{};
            }

            inline void return_void() {}

            inline void unhandled_exception() {
                std::terminate();
            }

            T current_value;
        };
        using handle_type = std::coroutine_handle<promise_type>;

        struct iterator_end {};

        struct iterator {
            inline iterator(handle_type handle_)
                : handle(handle_) {
            }

            inline void operator++() {
                handle.resume();
            }

            inline T operator*() {
                return std::move(handle.promise().current_value);
            }

            inline bool operator==(iterator_end) {
                return handle.done();
            }

            inline bool operator!=(iterator_end) {
                return !handle.done();
            }

            handle_type handle;
        };

        inline Generator(handle_type handle_)
            : handle(handle_) {
        }
        inline Generator(const Generator& other) = delete;
        inline Generator(Generator&& other) noexcept
            : handle(other.handle) {
            other.handle = nullptr;
        }
        inline ~Generator() {
            if (handle) {
                handle.destroy();
            }
        }

        inline bool has_next() {
            if (handle) {
                handle.resume();

                return !handle.done();
            }

            return false;
        }

        inline T get_value() {
            return std::move(handle.promise().current_value);
        }

        inline iterator begin() {
            handle.resume();
            return iterator{ handle };
        }

        inline iterator_end end() {
            return iterator_end{};
        }

    private:
        handle_type handle;
    };


}