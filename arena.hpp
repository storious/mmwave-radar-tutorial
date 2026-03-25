#ifndef ARENA_HPP_
#define ARENA_HPP_

#include <array>
#include <memory_resource>

template <typename T> class Arena {
public:
  using Region = std::pmr::monotonic_buffer_resource;

  Arena() = default;

  T *allocate(std::size_t size_) {
    return static_cast<T *>(::operator new(size_ * sizeof(T)));
  }

  void deallocate(T *ptr, std::size_t size_) { ::operator delete(ptr, size_); }

private:
  std::array<std::byte, 32> buffer;
};

#endif // ARENA_HPP_
