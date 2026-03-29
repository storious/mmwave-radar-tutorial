#include <stdlib.h>
#define ARENA_IMPLEMENTATION
#define UTEST_IMPLEMENTATION
#include "arena.h"
#include "utest.h"

static Arena default_arena = {0};
static Arena temporary_arena = {0};
static Arena *context_arena = &default_arena;

void *context_alloc(size_t size) {
  assert(context_arena);
  return arena_alloc(context_arena, size);
}

TEST(test_arena) {

  // Allocate stuff in default_arena
  EXPECT_NOT_NULLPTR(context_alloc(64));
  EXPECT_NOT_NULLPTR(context_alloc(128));
  EXPECT_NOT_NULLPTR(context_alloc(256));
  EXPECT_NOT_NULLPTR(context_alloc(512));

  // Allocate stuff in temporary_arena;
  context_arena = &temporary_arena;
  EXPECT_NOT_NULLPTR(context_alloc(64));
  EXPECT_NOT_NULLPTR(context_alloc(128));
  EXPECT_NOT_NULLPTR(context_alloc(256));
  EXPECT_NOT_NULLPTR(context_alloc(512));

  // Deallocate everything at once
  arena_free(&default_arena);
  arena_free(&temporary_arena);
  return EXIT_SUCCESS;
}

int main(void) {
  RUN_ALL;
  return EXIT_SUCCESS;
}
