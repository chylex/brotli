#ifndef VSB_DEBUG_OUTPUT_H_
#define VSB_DEBUG_OUTPUT_H_

#include <stdio.h>
#include <ctype.h>

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

#define DBG_CHAR(var) \
  if (!isprint(var)){ \
    DBG(#var " = \\%02d", var) \
  } else { \
    DBG(#var " = %c", var) \
  }

#define DBG_P_CHAR(prefix, var) \
  if (!isprint(var)){ \
    DBG(prefix " " #var " = \\%02d", var) \
  } else { \
    DBG(prefix " " #var " = %c", var) \
  }

#define DBG_P_OFFCHAR(prefix, pointer, base) \
  if (!isprint(*pointer)){ \
    DBG(prefix " %3lld (\\%02d)", (pointer - base), *pointer) \
  } else { \
    DBG(prefix " %3lld (%c)", (pointer - base), *pointer) \
  }

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

// String (no indentation)

#define DBG_STR(start, end) \
  unsigned char const* index = start; \
  while(index != end){ printf("%c", *index); ++index; } \
  printf("\n");

#define DBG_P_STR(prefix, start, end) \
  printf(prefix " "); \
  DBG_STR(start, end)

// Array (no indentation)

#define _DBG_ARR_PRINT(index_type, data_type, ptr, index) \
  if (index_type[strlen(index_type) - 1] == 'c' && !isprint(index)){ \
    printf("\\%02d : " data_type "\n", index, ptr[index]); \
  } else { \
    printf(index_type " : " data_type "\n", index, ptr[index]); \
  }

#define DBG_ARR(ptr, start, end, index_type, data_type) \
  for(int index = start; index <= end; index++){ \
    _DBG_ARR_PRINT(index_type, data_type, ptr, index) \
  }

#define DBG_ARR_COND(ptr, size, index_type, data_type, condition) \
  for(int index = 0; index < size; index++){ \
    if (condition){ _DBG_ARR_PRINT(index_type, data_type, ptr, index) } \
  }

#endif /* VSB_DEBUG_OUTPUT_H_ */
