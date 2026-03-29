#ifndef STRVIEW_H_
#define STRVIEW_H_
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ===============
// string view
// ===============

typedef struct {
  const char *data;
  size_t len;
} str_view, *str_view_ptr;

static inline str_view sv_from_cstr(const char *s) {
  return (str_view){.data = s, .len = s ? strlen(s) : 0};
}

#define SV(literal) ((str_view){.data = (literal), .len = sizeof(literal) - 1})

// slice [start, end)
static inline str_view sv_slice(str_view sv, size_t start, size_t end) {
  if (start > sv.len)
    start = sv.len;
  if (end > sv.len)
    end = sv.len;
  return (str_view){.data = sv.data + start, .len = end - start};
}

// remove prefix
static inline void sv_remove_prefix(str_view *sv, size_t n) {
  if (n > sv->len) {
    n = sv->len;
  }
  sv->data += n;
  sv->len -= n;
}

// compare
static inline bool sv_eq(str_view lhs, str_view rhs) {
  return lhs.len == rhs.len &&
         (lhs.data == rhs.data || memcmp(lhs.data, rhs.data, lhs.len) == 0);
}

// prefix start
static inline bool sv_starts_with(str_view sv, str_view prefix) {
  return sv.len >= prefix.len && memcmp(sv.data, prefix.data, prefix.len) == 0;
}

// debug
static inline void sv_print(str_view sv) {
  printf("%*.s", (int)sv.len, sv.data);
}

// ========================
// string buffer
// ========================

typedef struct {
  char *data;
  size_t len;
  size_t cap;
} str_buf;

static inline str_buf sb_create(size_t cap) {
  str_buf sb = {0};
  sb.data = (char *)malloc(cap);
  if (sb.data) {
    sb.data[0] = '\0';
    sb.cap = cap;
    sb.len = 0;
  }
  return sb;
}

// free
static inline void sb_free(str_buf *sb) {
  if (sb->data)
    free(sb->data);
  sb->data = nullptr;
  sb->len = 0;
  sb->cap = 0;
}

// extend (internal)
static inline void sb_ensure_cap(str_buf *sb, size_t needed) {
  if (needed <= sb->cap)
    return;
  size_t new_cap = sb->cap ? sb->cap * 2 : 64;
  while (new_cap < needed)
    new_cap *= 2;

  char *new_data = (char *)realloc(sb->data, new_cap);
  if (new_data) {
    sb->data = new_data;
    sb->cap = new_cap;
  }
}

// append string view
static inline void sb_append_sv(str_buf *sb, str_view sv) {
  size_t needed = sb->len + sv.len + 1;
  sb_ensure_cap(sb, needed);

  memcpy(sb->data + sb->len, sv.data, sv.len);
  sb->len += sv.len;
  sb->data[sb->len] = '\0';
}

// append C-str
static inline void sb_append_cstr(str_buf *sb, const char *s) {
  sb_append_sv(sb, sv_from_cstr(s));
}

#endif // !STRVIEW_H_
