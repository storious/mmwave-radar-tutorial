/*
================================================================================
    utest_test.c - Unit Tests for UTEST Library
================================================================================
*/

// Must define this in EXACTLY one source file
#define UTEST_IMPLEMENTATION
#include "utest.h"

// ==========================================
// 1. Basic Assertions & Type Coverage
// ==========================================

TEST(basic_assertions) {
  EXPECT_TRUE(1 == 1);
  EXPECT_TRUE(1, "Logic works");
  return 0;
}

TEST(eq_types_int) {
  int a = 100;
  EXPECT_EQ(a, 100);
  return 0;
}

TEST(eq_types_uint) {
  unsigned int a = 4000000000u;
  EXPECT_EQ(a, 4000000000u); // Tests %u format
  return 0;
}

TEST(eq_types_long) {
  long a = 123456789L;
  EXPECT_EQ(a, 123456789L);
  return 0;
}

TEST(eq_types_size_t) {
  size_t a = 1024;
  EXPECT_EQ(a, (size_t)1024); // Tests %zu format
  return 0;
}

TEST(eq_types_pointer) {
  int val = 10;
  int *ptr = &val;
  EXPECT_EQ(ptr, &val); // Tests %p format
  return 0;
}

// ==========================================
// 2. Float/Double Precision
// ==========================================

TEST(float_precision) {
  float a = 1.001f;
  EXPECT_NEAR(a, 1.0f, 0.01f, "Float tolerance check");
  return 0;
}

TEST(double_precision) {
  double a = 3.14159265;
  EXPECT_NEAR(a, 3.14159260, 1e-7, "Double tolerance check");
  return 0;
}

TEST(long_double_precision) {
  long double a = 3.14159265;
  EXPECT_NEAR(a, 3.14159260, 1e-7, "Long double tolerance check");
  return 0;
}

TEST(int_range_as_near) {
  // EXPECT_NEAR works for integers too (via _Generic promotion)
  
  // Int 
  EXPECT_NEAR(100, 102, 5);
 
  long l1 = 100, l2 = 102;
  EXPECT_NEAR(l1, l2, 5);

  long long ll1 = 100, ll2 = 102;
  EXPECT_NEAR(ll1, ll2, 5);


  return 0;
}

// ==========================================
// 3. String & Memory Comparison
// ==========================================

TEST(string_compare_equal) {
  const char *s1 = "Radar Imaging";
  const char *s2 = "Radar Imaging";
  EXPECT_STREQ(s1, s2);
  return 0;
}

TEST(string_compare_null) {
  // Both null should pass
  EXPECT_STREQ(nullptr, nullptr);
  return 0;
}

TEST(memory_compare_struct) {
  struct point {
    int x;
    int y;
  };

  struct point p1 = {10, 20};
  struct point p2 = {10, 20};

  EXPECT_MEMEQ(&p1, &p2, sizeof(struct point));
  return 0;
}

TEST(memory_compare_array) {
  int data1[] = {1, 2, 3, 4, 5};
  int data2[] = {1, 2, 3, 4, 5};

  EXPECT_MEMEQ(data1, data2, sizeof(data1));
  return 0;
}

// ==========================================
// 4. Pointer Checks
// ==========================================

TEST(pointer_checks) {
  void *ptr = test_malloc(64);
  EXPECT_NOT_NULLPTR(ptr, "Allocation should succeed");

  test_free(ptr);

  void *null_ptr = nullptr;
  EXPECT_NULLPTR(null_ptr);
  return 0;
}

// ==========================================
// 5. Memory Management & Leak Detection
// ==========================================

TEST(alloc_free_no_leak) {
  void *p1 = test_malloc(128);
  void *p2 = test_malloc(256);

  EXPECT_NOT_NULLPTR(p1);
  EXPECT_NOT_NULLPTR(p2);

  test_free(p1);
  test_free(p2);
  // If we forget to free, RUN_TESTS will report [LEAK]
  return 0;
}

TEST(oom_simulation) {
  // Simulate Out-Of-Memory on the next allocation
  test_simulate_oom(1);

  void *p = test_malloc(10);
  
  // This allocation should fail
  EXPECT_NULLPTR(p, "Allocation should fail under OOM simulation");

  // Reset counter manually if needed (utest_reset_allocator also clears it)
  utest_mem_fail_counter = 0;
  return 0;
}

// ==========================================
// 6. Demonstration of Failures (Commented Out)
// ==========================================

// Uncomment these to see what failure output looks like
/*
TEST(demo_fail_eq) {
  int status = 0;
  EXPECT_EQ(status, 1, "Intentional failure for demo");
  return 0;
}

TEST(demo_fail_leak) {
  void *p = test_malloc(100);
  // Intentionally not freeing 'p' to trigger leak report
  return 0;
}

TEST(demo_fail_streq) {
  EXPECT_STREQ("Target A", "Target B", "String mismatch demo");
  return 0;
}
*/

// ==========================================
// Main Entry Point
// ==========================================

int main(int argc, char **argv) {
  // Pass command line args to filter tests if needed
  // e.g. ./test_utest eq_types
  RUN_TESTS(argc, argv);
}

