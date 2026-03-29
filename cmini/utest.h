/*
================================================================================
    UTEST - Lightweight Unit Testing Library with Internal Memory Pool

    Features:
    - Single header (STB style)
    - Internal Arena-like pool for zero-overhead memory tracking
    - No conflicts with external arena.h libraries
    - Auto-registration & Leak detection

    Usage:
        In ONE source file, define UTEST_IMPLEMENTATION before including:
            #define UTEST_IMPLEMENTATION
            #include "utest.h"
================================================================================
*/

#ifndef UTEST_H
#define UTEST_H

#include <math.h> // IWYU pragma: keep
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==========================================
// 1. Public API
// ==========================================

typedef int (*test_func_t)(void);

typedef struct test_case {
  const char *name;
  test_func_t func;
  struct test_case *next;
} test_case_t;

extern test_case_t *utest_list_head;
extern int utest_mem_fail_counter;

void utest_register(test_case_t *tc);
void *utest_malloc_internal(size_t size, const char *file, int line);
void utest_free_internal(void *ptr, const char *file, int line);
void utest_reset_allocator(void);
int utest_check_leaks(void);

// ================== internal ====================
static bool _utest_should_run(const char *name, int argc, char **argv) {
  if (argc <= 1)
    return true;
  for (int i = 1; i < argc; i++) {
    // match prefix for test_case
    if (strncmp(name, argv[i], strlen(argv[i])) == 0) {
      return true;
    }
  }
  return false;
}

// ==========================================
// 2. User Macros
// ==========================================

#define TEST(test_case_name)                                                   \
  int test_func_##test_case_name(void);                                        \
  __attribute__((constructor)) static void register_func_##test_case_name(     \
      void) {                                                                  \
    static test_case_t tc = {.name = #test_case_name,                          \
                             .func = test_func_##test_case_name};              \
    utest_register(&tc);                                                       \
  }                                                                            \
  int test_func_##test_case_name(void)

#define test_malloc(size) utest_malloc_internal(size, __FILE__, __LINE__)
#define test_free(ptr) utest_free_internal(ptr, __FILE__, __LINE__)
#define test_simulate_oom(n) (utest_mem_fail_counter = n)

// ==========================================
// 3. Assert Macros
// ==========================================

#define _UT_FAIL(msg, file, line, ...)                                         \
  do {                                                                         \
    fprintf(stderr, "  [FAIL] %s\n", msg);                                     \
    __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)            \
    fprintf(stderr, "        [AT] %s:%d\n", file, line);                       \
  } while (0);

#define EXPECT_TRUE(expr, ...)                                                 \
  do {                                                                         \
    if (!(expr)) {                                                             \
      _UT_FAIL(#expr, __FILE__, __LINE__ __VA_OPT__(, __VA_ARGS__));           \
      return 1;                                                                \
    }                                                                          \
  } while (0);

// Type-aware format selector (C11 _Generic)
#define _UT_FMT(val)                                                           \
  _Generic((val),                                                              \
      char: "%d",                                                              \
      unsigned char: "%u",                                                     \
      short: "%d",                                                             \
      unsigned short: "%u",                                                    \
      int: "%d",                                                               \
      unsigned int: "%u",                                                      \
      long: "%ld",                                                             \
      unsigned long: "%lu",                                                    \
      long long: "%lld",                                                       \
      unsigned long long: "%llu",                                              \
      float: "%g",                                                             \
      double: "%g",                                                            \
      long double: "%Lg",                                                      \
      default: "%p")

#define EXPECT_EQ(expected, actual, ...)                                       \
  do {                                                                         \
    typeof(expected) _e = (expected);                                          \
    typeof(actual) _a = (actual);                                              \
    if (_e != _a) {                                                            \
      fprintf(stderr, "  [FAIL] %s == %s\n", #expected, #actual);              \
      fprintf(stderr, "        Values: ");                                     \
      fprintf(stderr, _UT_FMT(_e), _e);                                        \
      fprintf(stderr, " != ");                                                 \
      fprintf(stderr, _UT_FMT(_a), _a);                                        \
      fprintf(stderr, "\n");                                                   \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    }                                                                          \
  } while (0);

#define EXPECT_STREQ(expected, actual, ...)                                    \
  do {                                                                         \
    const char *_e = (expected);                                               \
    const char *_a = (actual);                                                 \
    /* Handle null pointer cases */                                            \
    if (_e == nullptr && _a == nullptr) { /* both null, pass */                \
    } else if (_e == nullptr || _a == nullptr) {                               \
      fprintf(stderr, "  [FAIL] %s == %s\n", #expected, #actual);              \
      fprintf(stderr, "        Values: (null) != \"%s\"\n", _e ? _e : _a);     \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    } else if (strcmp(_e, _a) != 0) {                                          \
      fprintf(stderr, "  [FAIL] %s == %s\n", #expected, #actual);              \
      fprintf(stderr, "        Values: \"%s\" != \"%s\"\n", _e, _a);           \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    }                                                                          \
  } while (0);

#define EXPECT_MEMEQ(expected, actual, size, ...)                              \
  do {                                                                         \
    const void *_e = (expected);                                               \
    const void *_a = (actual);                                                 \
    size_t _s = (size);                                                        \
    if (_e == nullptr && _a == nullptr) { /* both null, pass */                \
    } else if (_e == nullptr || _a == nullptr) {                               \
      fprintf(stderr, "  [FAIL] %s == %s (size %zu)\n", #expected, #actual,    \
              _s);                                                             \
      fprintf(stderr, "        One pointer is null\n");                        \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    } else if (memcmp(_e, _a, _s) != 0) {                                      \
      fprintf(stderr, "  [FAIL] %s == %s (size %zu)\n", #expected, #actual,    \
              _s);                                                             \
      /* Optional: hex dump first few bytes */                                 \
      fprintf(stderr, "        Memory content differs\n");                     \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    }                                                                          \
  } while (0);

// Select correct fabs function based on type
#define _UT_ABS(v)                                                             \
  _Generic((v),                                                                \
      int: abs,                                                                \
      long: labs,                                                              \
      long long: llabs,                                                        \
      float: fabsf,                                                            \
      long double: fabsl,                                                      \
      default: fabs)(v) // default handles double, int, etc.

#define EXPECT_NEAR(expected, actual, tolerance, ...)                          \
  do {                                                                         \
    typeof(expected) _e = (expected);                                          \
    typeof(actual) _a = (actual);                                              \
    typeof(tolerance) _t = (tolerance);                                        \
                                                                               \
    /* Use type-respecting fabs, then compare against tolerance */             \
    if (_UT_ABS(_e - _a) > _t) {                                               \
      fprintf(stderr, "  [FAIL] |%s - %s| > %s\n", #expected, #actual,         \
              #tolerance);                                                     \
      fprintf(stderr, "        Diff: ");                                       \
      fprintf(stderr, _UT_FMT(_e), _e);                                        \
      fprintf(stderr, " != ");                                                 \
      fprintf(stderr, _UT_FMT(_a), _a);                                        \
      fprintf(stderr, " (tol: ");                                              \
      fprintf(stderr, _UT_FMT(_t), _t);                                        \
      fprintf(stderr, ")\n");                                                  \
      __VA_OPT__(fprintf(stderr, "        [MSG] %s\n", __VA_ARGS__);)          \
      fprintf(stderr, "        [AT] %s:%d\n", __FILE__, __LINE__);             \
      return 1;                                                                \
    }                                                                          \
  } while (0);

#define EXPECT_NULLPTR(ptr, ...) EXPECT_TRUE((ptr) == nullptr, __VA_ARGS__)
#define EXPECT_NOT_NULLPTR(ptr, ...) EXPECT_TRUE((ptr) != nullptr, __VA_ARGS__)

// ==========================================
// 4. Runner
// ==========================================

#define RUN_TESTS(argc, argv)                                                  \
  do {                                                                         \
    int passed = 0, failed = 0, skipped = 0;                                   \
    printf("\n[START] Running Tests...\n\n");                                  \
                                                                               \
    test_case_t *current = utest_list_head;                                    \
    while (current) {                                                          \
      /* filter logical：if prefix not match skipped current test_case */      \
      if (!_utest_should_run(current->name, argc, argv)) {                     \
        current = current->next;                                               \
        skipped++;                                                             \
        continue;                                                              \
      }                                                                        \
                                                                               \
      printf("-> Running: %s ... ", current->name);                            \
                                                                               \
      utest_reset_allocator();                                                 \
      int result = current->func();                                            \
                                                                               \
      if (result == 0) {                                                       \
        int leaks = utest_check_leaks();                                       \
        if (leaks > 0) {                                                       \
          printf("[LEAK] Memory Leaked\n");                                    \
          failed++;                                                            \
        } else {                                                               \
          printf("[PASS]\n");                                                  \
          passed++;                                                            \
        }                                                                      \
      } else {                                                                 \
        utest_check_leaks();                                                   \
        printf("[FAIL]\n");                                                    \
        failed++;                                                              \
      }                                                                        \
                                                                               \
      current = current->next;                                                 \
    }                                                                          \
                                                                               \
    printf("\n-------------------\n");                                         \
    printf("Total: %d | [PASS]: %d | [FAIL]: %d | [SKIP]: %d\n",               \
           passed + failed + skipped, passed, failed, skipped);                \
    if (failed)                                                                \
      return failed;                                                           \
  } while (false);

#define RUN_ALL RUN_TESTS(0, nullptr)

// ========================================================================
// IMPLEMENTATION
// ========================================================================
#ifdef UTEST_IMPLEMENTATION

// ==========================================
// Internal Memory Pool (Hidden Arena)
// ==========================================

// Internal record structure
typedef struct _utest_mem_record {
  void *ptr;
  size_t size;
  const char *file;
  int line;
  bool freed;
} _utest_mem_record_t;

// Internal memory chunk (Arena-like block)
typedef struct _utest_chunk {
  struct _utest_chunk *next;
  size_t count;
  size_t capacity;
  _utest_mem_record_t data[];
} _utest_chunk_t;

#ifndef UTEST_POOL_CAPACITY
#define UTEST_POOL_CAPACITY (1024) // Records per chunk
#endif

static _utest_chunk_t *_utest_pool_head = nullptr;
static _utest_chunk_t *_utest_pool_active = nullptr;

// Allocate a record from the pool (Bump Allocation)
static _utest_mem_record_t *_utest_pool_alloc(void) {
  if (!_utest_pool_active ||
      _utest_pool_active->count >= _utest_pool_active->capacity) {

    // Allocate new chunk
    size_t chunk_size = sizeof(_utest_chunk_t) +
                        UTEST_POOL_CAPACITY * sizeof(_utest_mem_record_t);
    _utest_chunk_t *new_chunk = (_utest_chunk_t *)malloc(chunk_size);
    if (!new_chunk)
      return nullptr; // OOM

    new_chunk->count = 0;
    new_chunk->capacity = UTEST_POOL_CAPACITY;

    // Link to list (LIFO for cache locality)
    new_chunk->next = _utest_pool_head;
    _utest_pool_head = new_chunk;
    _utest_pool_active = new_chunk;
  }

  return &_utest_pool_active->data[_utest_pool_active->count++];
}

// Reset the pool (Fast O(1) logically, just reset counters)
static void _utest_pool_reset(void) {
  _utest_chunk_t *chunk = _utest_pool_head;
  while (chunk) {
    chunk->count = 0;
    chunk = chunk->next;
  }
  _utest_pool_active = _utest_pool_head; // Reuse existing blocks
}

// Free all chunks (Called only at program exit if needed,
// but utest usually runs in main, so OS cleans up.
// Included for completeness.)
static void _utest_pool_free_all(void) {
  _utest_chunk_t *chunk = _utest_pool_head;
  while (chunk) {
    _utest_chunk_t *next = chunk->next;
    free(chunk);
    chunk = next;
  }
  _utest_pool_head = nullptr;
  _utest_pool_active = nullptr;
}

// ==========================================
// Global State
// ==========================================

test_case_t *utest_list_head = nullptr;
int utest_mem_fail_counter = 0;

void utest_register(test_case_t *tc) {
  tc->next = utest_list_head;
  utest_list_head = tc;
}

// ==========================================
// Implementation Functions
// ==========================================

void *utest_malloc_internal(size_t size, const char *file, int line) {
  if (utest_mem_fail_counter > 0) {
    utest_mem_fail_counter--;
    return nullptr;
  }

  // 1. Allocate metadata from Internal Pool (Super Fast)
  _utest_mem_record_t *rec = _utest_pool_alloc();
  if (!rec)
    return nullptr;

  // 2. Allocate user memory from System Heap
  void *ptr = malloc(size);
  if (!ptr) {
    // If user malloc fails, we technically "leak" the pool record,
    // but it stays in the pool. This is acceptable for testing.
    return nullptr;
  }

  rec->ptr = ptr;
  rec->size = size;
  rec->file = file;
  rec->line = line;
  rec->freed = false;

  return ptr;
}

void utest_free_internal(void *ptr, const char *file, int line) {
  if (!ptr)
    return;

  // Iterate through the pool chunks to find the record
  // Note: We only need to scan the active chunks used so far.
  _utest_chunk_t *chunk = _utest_pool_head;

  // Optimization: We only need to scan chunks that have data.
  // Since chunks are linked LIFO, we scan current active first.
  while (chunk) {
    for (size_t i = 0; i < chunk->count; i++) {
      if (chunk->data[i].ptr == ptr) {
        if (chunk->data[i].freed) {
          fprintf(stderr, "[ERROR] Double Free detected at %s:%d\n", file,
                  line);
          fprintf(stderr, "        Originally allocated at %s:%d\n",
                  chunk->data[i].file, chunk->data[i].line);
          return;
        }
        chunk->data[i].freed = true;
        free(ptr); // Free system memory
        return;
      }
    }
    chunk = chunk->next;
  }
  fprintf(stderr, "[ERROR] Invalid Free (untracked pointer) at %s:%d\n", file,
          line);
}

void utest_reset_allocator(void) {
  // 1. Cleanup leaked system memory
  _utest_chunk_t *chunk = _utest_pool_head;
  while (chunk) {
    for (size_t i = 0; i < chunk->count; i++) {
      if (!chunk->data[i].freed) {
        free(chunk->data[i].ptr);
      }
    }
    chunk = chunk->next;
  }

  // 2. Reset pool (Instant, no system calls for metadata)
  _utest_pool_reset();
  utest_mem_fail_counter = 0;
}

int utest_check_leaks(void) {
  int leaks = 0;
  _utest_chunk_t *chunk = _utest_pool_head;
  while (chunk) {
    for (size_t i = 0; i < chunk->count; i++) {
      if (!chunk->data[i].freed) {
        fprintf(stderr, "[ERROR] Memory Leak: %zu bytes at %p\n",
                chunk->data[i].size, chunk->data[i].ptr);
        fprintf(stderr, "        Allocated at %s:%d\n", chunk->data[i].file,
                chunk->data[i].line);
        leaks++;
      }
    }
    chunk = chunk->next;
  }
  return leaks;
}

#endif // UTEST_IMPLEMENTATION

#endif // UTEST_H
