#ifndef RESULT_H_
#define RESULT_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { RESULT_OK, RESULT_ERR } ResultTag;

#define DEFINE_RESULT(T, E)                                                    \
  typedef struct {                                                             \
    ResultTag tag;                                                             \
    union {                                                                    \
      T ok;                                                                    \
      E err;                                                                   \
    } value;                                                                   \
  } Result_##T##_##E;                                                          \
                                                                               \
  static inline Result_##T##_##E result_ok_##T##_##E(T val) {                  \
    Result_##T##_##E res;                                                      \
    res.tag = RESULT_OK;                                                       \
    res.value.ok = val;                                                        \
    return res;                                                                \
  }                                                                            \
                                                                               \
  static inline Result_##T##_##E result_err_##T##_##E(E err) {                 \
    Result_##T##_##E res;                                                      \
    res.tag = RESULT_ERR;                                                      \
    res.value.err = err;                                                       \
    return res;                                                                \
  }                                                                            \
                                                                               \
  static inline bool is_ok_##T##_##E(Result_##T##_##E *res) {                  \
    return res != NULL && res->tag == RESULT_OK;                               \
  }                                                                            \
                                                                               \
  static inline bool is_err_##T##_##E(Result_##T##_##E *res) {                 \
    return res != NULL && res->tag == RESULT_ERR;                              \
  }                                                                            \
                                                                               \
  static inline T unwrap_##T##_##E(Result_##T##_##E *res) {                    \
    if (!is_ok_##T##_##E(res)) {                                               \
      fprintf(stderr, "[Result] unwrap failed: error state\n");                \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    return res->value.ok;                                                      \
  }                                                                            \
                                                                               \
  static inline E unwrap_err_##T##_##E(Result_##T##_##E *res) {                \
    if (!is_err_##T##_##E(res)) {                                              \
      fprintf(stderr, "[Result] unwrap_err failed：success state\n");          \
      exit(EXIT_FAILURE);                                                      \
    }                                                                          \
    return res->value.err;                                                     \
  }

typedef const char *EStr;
DEFINE_RESULT(int, EStr)
#define ResultIntErrStr Result_int_EStr
#define OkInt(val) result_ok_int_EStr(val)
#define ErrStr(msg) result_err_int_EStr(msg)
#define isOkInt(res) is_ok_int_EStr(res)
#define isErrInt(res) is_err_int_EStr(res)
#define unwrapInt(res) unwrap_int_EStr(res)
#define unwrapErrStr(res) unwrap_err_int_EStr(res)

DEFINE_RESULT(double, int)
#define ResultDoubleErrInt Result_double_int
#define OkDouble(val) result_ok_double_int(val)
#define ErrInt(code) result_err_double_int(code)

typedef void *Any;
DEFINE_RESULT(Any, int)
#define ResultPtrErrInt Result_Any_int
#define OkPtr(val) result_ok_Any_int(val)
#define ErrPtr(code) result_err_Any_int(code)

#endif // !RESULT_H_
