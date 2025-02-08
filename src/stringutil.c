#include <stdbool.h>
#include <string.h>

bool starts_with(const char *src, const char *prefix, char **after) {
  int len = strlen(prefix);
  if (strncmp(src, prefix, len) == 0) {
    *after = (char *)src + len;
    return true;
  }
  return false;
}
