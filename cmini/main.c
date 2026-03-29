#define UTEST_IMPLEMENTATION
#include <stdlib.h>
#include <string.h>

#include "utest.h"

// 2.2
/**
 * @author: wujie
 * @param: const char* src, source string pointer
 * @param: char* dst, destination string pointer, the first `sep` character
 * separated word
 * @param: const char sep, the separated character, such as " ", ",", "\"
 * @return: size_t, the size of the first `sep` character separated word
 */
size_t strsplit(const char *src, char **dst, const char sep);

// 2.2 implementation

size_t strsplit(const char *src, char **dst_ptr, const char sep) {
  size_t len = strlen(src);
  for (size_t pos = 0; pos < len; ++pos) {
    if (src[pos] == sep) {
      (*dst_ptr) = (char *)malloc(pos + 1);
      memcpy((*dst_ptr), src, pos);
      (*dst_ptr)[pos] = '\0';
      return pos;
    }
  }
  // if not found, copy whole string
  (*dst_ptr) = (char *)malloc(len + 1);
  memcpy((*dst_ptr), src, len);
  (*dst_ptr)[len] = '\0';
  return len;
}

TEST(test_strsplit) {
  char *src = "hello world";
  char *dst;
  size_t length = strsplit(src, &dst, ' ');
  EXPECT_EQ(length, 5);
  EXPECT_EQ(strcmp(dst, "hello"), 0, "dst is not equal to `hello`");
  free(dst);

  src = "C:/User/Projects";
  length = strsplit(src, &dst, '/');
  EXPECT_EQ(length, 2);
  EXPECT_EQ(length, strlen(dst));
  EXPECT_EQ(strcmp(dst, "C:"), 0, "dst is not equal to `C:`");
  free(dst);

  src = "hello\0";
  length = strsplit(src, &dst, '\0');
  EXPECT_EQ(length, 5);
  EXPECT_EQ(length, strlen(dst));
  EXPECT_EQ(strcmp(dst, "hello"), 0, "dst is not equal to `hello`");
  free(dst);

  src = "hello";
  length = strsplit(src, &dst, ' ');
  EXPECT_EQ(length, 5);
  EXPECT_EQ(length, strlen(dst));
  EXPECT_EQ(strcmp(dst, "hello"), 0, "dst is not equal to `hello`");
  free(dst);

  src = "hello ";
  length = strsplit(src, &dst, ' ');
  EXPECT_EQ(length, 5);
  EXPECT_EQ(length, strlen(dst));
  EXPECT_EQ(strcmp(dst, "hello"), 0, "dst is not equal to `hello`");
  free(dst);
  return EXIT_SUCCESS;
}

int main(void) {
  RUN_ALL;
  return EXIT_SUCCESS;
}
