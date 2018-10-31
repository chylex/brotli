#ifndef VSB_DEBUG_OUTPUT_H_
#define VSB_DEBUG_OUTPUT_H_

#include <stdio.h>

int DBG_SPACES;

// Basic output

#define DBG(format, ...) \
  printf("%*s", DBG_SPACES, ""); \
  printf(format, __VA_ARGS__); \
  printf("\n");

#define DBG_VAR(var, type) \
  DBG(#var " = %" #type, var)

#define DBG_P_VAR(prefix, var, type) \
  DBG(prefix " " #var " = %" #type, var)

#define DBG_P_OFFCHAR(prefix, pointer, base) \
  DBG(prefix " %3lld (%c)", (pointer - base), *pointer)

// Flow control

#define DBG_RESET_ \
  DBG_SPACES = 0;

#define DBG_START_ \
  ++DBG_SPACES;

#define DBG_END_ \
  --DBG_SPACES;

#define DBG_START(message) \
  DBG("> [" message "]") \
  DBG_START_

#define DBG_END(message) \
  DBG_END_ \
  DBG("< [" message "]")

#endif /* VSB_DEBUG_OUTPUT_H_ */
