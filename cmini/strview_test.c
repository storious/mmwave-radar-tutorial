
/*
================================================================================
    utest_strview.c - Unit Tests for STRVIEW Library
================================================================================
*/

// Must define this in EXACTLY one source file
#include <stdlib.h>
#define UTEST_IMPLEMENTATION
#include "utest.h"

// Include the header to test
#include "strview.h"

// ==========================================
// 1. String View Creation & Basic
// ==========================================

TEST(sv_from_cstr_basic) {
  const char *s = "Hello World";
  str_view sv = sv_from_cstr(s);

  EXPECT_EQ(sv.len, 11);
  EXPECT_EQ(sv.data, s);
  return 0;
}

TEST(sv_from_cstr_empty) {
  str_view sv = sv_from_cstr("");
  EXPECT_EQ(sv.len, 0);
  return 0;
}

TEST(sv_from_cstr_null) {
  str_view sv = sv_from_cstr(NULL);
  EXPECT_EQ(sv.len, 0);
  EXPECT_TRUE(sv.data == NULL);
  return 0;
}

TEST(sv_literal_macro) {
  // SV macro should calculate length at compile time (sizeof - 1)
  str_view sv = SV("Test");
  EXPECT_EQ(sv.len, 4);
  // data should point to the literal
  EXPECT_TRUE(sv.data != NULL);
  return 0;
}

// ==========================================
// 2. String View Slicing
// ==========================================

TEST(sv_slice_middle) {
  str_view sv = SV("Hello World");
  // Slice "World"
  str_view slice = sv_slice(sv, 6, 11);
  EXPECT_EQ(slice.len, 5);
  EXPECT_MEMEQ(slice.data, "World", 5);
  return 0;
}

TEST(sv_slice_start) {
  str_view sv = SV("Hello World");
  // Slice "Hello"
  str_view slice = sv_slice(sv, 0, 5);
  EXPECT_EQ(slice.len, 5);
  EXPECT_MEMEQ(slice.data, "Hello", 5);
  return 0;
}

TEST(sv_slice_empty) {
  str_view sv = SV("Hello");
  // Zero length slice
  str_view slice = sv_slice(sv, 2, 2);
  EXPECT_EQ(slice.len, 0);
  return 0;
}

TEST(sv_slice_out_of_bounds) {
  str_view sv = SV("Short");
  // End exceeds length, should clamp to len
  str_view slice = sv_slice(sv, 0, 100);
  EXPECT_EQ(slice.len, 5); // Should be clamped to length of "Short"
  return 0;
}

TEST(sv_slice_invalid_start) {
  str_view sv = SV("Short");
  // Start exceeds length
  str_view slice = sv_slice(sv, 100, 101);
  EXPECT_EQ(slice.len, 0); // Should result in zero length
  return 0;
}

// ==========================================
// 3. String View Modification & Compare
// ==========================================

TEST(sv_remove_prefix_basic) {
  str_view sv = SV("TestData");
  sv_remove_prefix(&sv, 4); // Remove "Test"
  EXPECT_EQ(sv.len, 4);
  EXPECT_MEMEQ(sv.data, "Data", 4);
  return 0;
}

TEST(sv_remove_prefix_all) {
  str_view sv = SV("Data");
  sv_remove_prefix(&sv, 10); // Remove more than length
  EXPECT_EQ(sv.len, 0);
  return 0;
}

TEST(sv_eq_equal) {
  str_view s1 = SV("Compare");
  str_view s2 = SV("Compare");
  EXPECT_TRUE(sv_eq(s1, s2));
  return 0;
}

TEST(sv_eq_not_equal) {
  str_view s1 = SV("Compare");
  str_view s2 = SV("Compara");
  EXPECT_TRUE(!sv_eq(s1, s2));
  return 0;
}

TEST(sv_eq_different_len) {
  str_view s1 = SV("Short");
  str_view s2 = SV("Shorter");
  EXPECT_TRUE(!sv_eq(s1, s2));
  return 0;
}

TEST(sv_eq_same_pointer) {
  const char *literal = "SamePtr";
  str_view s1 = sv_from_cstr(literal);
  str_view s2 = sv_from_cstr(literal);
  // Optimization: pointers equal means content equal
  EXPECT_TRUE(sv_eq(s1, s2));
  return 0;
}

TEST(sv_starts_with_true) {
  str_view sv = SV("Configuration");
  EXPECT_TRUE(sv_starts_with(sv, SV("Conf")));
  return 0;
}

TEST(sv_starts_with_false) {
  str_view sv = SV("Configuration");
  EXPECT_TRUE(!sv_starts_with(sv, SV("Fig")));
  return 0;
}

TEST(sv_starts_with_longer_prefix) {
  str_view sv = SV("Short");
  str_view prefix = SV("LongerPrefix");
  EXPECT_TRUE(!sv_starts_with(sv, prefix));
  return 0;
}

// ==========================================
// 4. String Buffer Creation & Lifecycle
// ==========================================

TEST(sb_create_basic) {
  str_buf sb = sb_create(64);
  EXPECT_NOT_NULLPTR(sb.data);
  EXPECT_EQ(sb.cap, 64);
  EXPECT_EQ(sb.len, 0);
  EXPECT_EQ(sb.data[0], '\0'); // Should be empty string
  sb_free(&sb);
  return 0;
}

TEST(sb_free_works) {
  str_buf sb = sb_create(10);
  sb_free(&sb);
  EXPECT_TRUE(sb.data == NULL);
  EXPECT_EQ(sb.len, 0);
  EXPECT_EQ(sb.cap, 0);
  return 0;
}

// ==========================================
// 5. String Buffer Append & Growth
// ==========================================

TEST(sb_append_cstr_basic) {
  str_buf sb = sb_create(10);
  sb_append_cstr(&sb, "Hello");

  EXPECT_EQ(sb.len, 5);
  EXPECT_STREQ(sb.data, "Hello");

  sb_free(&sb);
  return 0;
}

TEST(sb_append_multiple) {
  str_buf sb = sb_create(10);
  sb_append_cstr(&sb, "Hello");
  sb_append_cstr(&sb, " ");
  sb_append_cstr(&sb, "World");

  EXPECT_EQ(sb.len, 11); // 5 + 1 + 5
  EXPECT_STREQ(sb.data, "Hello World");

  sb_free(&sb);
  return 0;
}

TEST(sb_append_sv) {
  str_buf sb = sb_create(5);
  str_view part = SV("123456789"); // Longer than initial cap

  sb_append_sv(&sb, part);

  EXPECT_EQ(sb.len, 9);
  EXPECT_STREQ(sb.data, "123456789");
  // Check if capacity grew (logic: initial 5 -> should grow to hold 10)
  EXPECT_TRUE(sb.cap >= 10);

  sb_free(&sb);
  return 0;
}

TEST(sb_growth_logic) {
  // Initial capacity 4
  str_buf sb = sb_create(4);

  // Append "123" (len 3 + null = 4) -> fits exactly
  sb_append_cstr(&sb, "123");
  EXPECT_EQ(sb.cap, 4);

  // Append "4" (len 4 + 1 = 5) -> needs growth
  sb_append_cstr(&sb, "4");
  EXPECT_EQ(sb.len, 4);
  EXPECT_TRUE(sb.cap > 4); // Capacity should have doubled or grown

  EXPECT_STREQ(sb.data, "1234");

  sb_free(&sb);
  return 0;
}

// ==========================================
// Main Entry Point
// ==========================================

int main(void) {
  // Pass command line args to filter tests if needed
  RUN_ALL;
  return EXIT_SUCCESS;
}
