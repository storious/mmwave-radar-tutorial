#define UTEST_IMPLEMENTATION
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "pointer.h"
#include "utest.h"

TEST(pointer) {
  unique_ptr(int) p1 = malloc(sizeof(int));
  auto_scoped p2 = make_unique(int);
  auto_scoped p3 = make_unique_array(int, 3);
  auto_scoped p4 = make_unique_zero(int);
  return 0;
}

typedef struct Person Person;

struct Person {
  char *name;
  size_t age;
  void *data;
  Person *next;
};

Person *person_new(char *name, size_t age) {
  Person *p = test_malloc(sizeof(Person));
  p->name = name;
  p->age = age;
  (*p) = (Person){.name = name, .age = age};
  return p;
}

void person_free(void *ptr) {
  Person *p = (Person *)ptr;
  if (p->data)
    test_free(p->data);
  test_free(p);
  printf("\ncustom free\n");
}

TEST(pointer_advance) {
  unique_ptr_custom(Person, person_free) p1 =
      make_unique_custom(Person, person_free, person_new("storious", 16));
  unique_ptr_custom(Person, person_free) p2 =
      make_unique_custom(Person, person_free, person_new("strius", 22));
  p2.ptr->data = test_malloc(sizeof(int));

  p2.ptr->next = test_malloc(sizeof(Person));
  test_free(p2.ptr->next);
  return 0;
}

int main(void) {
  RUN_ALL;
  return EXIT_SUCCESS;
}
