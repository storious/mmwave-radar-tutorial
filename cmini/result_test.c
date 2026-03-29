#include <stdlib.h>
#define UTEST_IMPLEMENTATION
#include "result.h"
#include "utest.h"

ResultIntErrStr divide(int a, int b) {
  if (b == 0) {
    return ErrStr("b is 0");
  }
  return OkInt(a / b);
}

TEST(test_result) {
  ResultIntErrStr res1 = divide(10, 2);
  if (isOkInt(&res1)) {
    printf("success: %d\n", unwrapInt(&res1));
  } else {
    printf("failed: %s\n", unwrapErrStr(&res1));
  }

  ResultIntErrStr res2 = divide(5, 0);
  if (isOkInt(&res2)) {
    printf("success: %d\n", unwrapInt(&res2));
  } else {
    printf("failed: %s\n", unwrapErrStr(&res2));
  }
  return EXIT_SUCCESS;
}

int main(void) {
  RUN_ALL;
  return 0;
}
