#ifndef POINTER_H_
#define POINTER_H_

#include <stdio.h>
#include <stdlib.h>

#ifndef NDEBUG
constexpr bool utest_debug_mode = true;
#else
constexpr bool utest_debug_mode = false;
#endif

// ==========================================
// 1. core cleanup function
// ==========================================

// default deleter
static inline void smart_free(void *ptr) {
  void **p = (void **)ptr;
  if (p && *p) {
    free(*p);
    *p = nullptr;
    if (utest_debug_mode)
      fprintf(stderr, "\nfree smart ptr\n");
  }
}

// ==========================================
// 2. basic define ( C23 typeof)
// ==========================================

// basic smart pointer define
#define unique_ptr(typename) __attribute__((cleanup(smart_free))) typename *

// C23：auto inference
// example: auto_scoped p = malloc(...);
#define auto_scoped __attribute__((cleanup(smart_free))) auto

// ==========================================
// 3. allocator helper define
// ==========================================

// single object allocator
// example: unique_ptr(int) p = make_unique(int);
#define make_unique(typename) ((typename *)malloc(sizeof(typename)))

// array allocator
// example: unique_ptr(int) arr = make_unique_array(int, 10);
#define make_unique_array(typename, count)                                     \
  ((typename *)malloc(sizeof(typename) * (count)))

// zero initialilze allocator
// example: unique_ptr(struct Radar) r = make_unique_zero(struct Radar);
#define make_unique_zero(typename) ((typename *)calloc(1, sizeof(typename)))

// ==========================================
// 4. advance feature：custom (support recurise struct)
// ==========================================

// general deleter
// @brief: allowed transfer a free function
typedef void (*deleter_func_t)(void *);

static inline void smart_free_custom(void *ptr) {
  struct control_block {
    void *raw_ptr;
    deleter_func_t deleter;
  } *cb = (struct control_block *)ptr;

  if (cb && cb->raw_ptr) {
    if (cb->deleter) {
      cb->deleter(cb->raw_ptr);
    } else {
      free(cb->raw_ptr); // default behavior
    }
    cb->raw_ptr = nullptr;
  }
}

// smart pointer define（custom deleter）
// example: unique_ptr_custom(MyStruct, my_deleter_func) p = ...
#define unique_ptr_custom(typename, deleter)                                   \
  __attribute__((cleanup(smart_free_custom))) struct {                         \
    typename *ptr;                                                             \
    deleter_func_t deleter;                                                    \
  }

// helper initialilze define
// example: auto p = make_unique_custom(MyStruct, my_deleter);
#define make_unique_custom(typename, deleter, raw_ptr)                     \
  {.ptr = (typename *)raw_ptr, .deleter = (deleter_func_t)deleter}

// ==========================================
// 5. quick cast (manage raw pointer)
// ==========================================

#define _SMART_CONCAT_(a, b) a##b
#define _SMART_CONCAT(a, b) _SMART_CONCAT_(a, b)

#define unique_ptr_cast(raw_ptr)                                               \
  __attribute__((cleanup(smart_free))) typeof(raw_ptr) _SMART_CONCAT(          \
      scoped_, __LINE__) = (raw_ptr)

#endif // !POINTER_H_
