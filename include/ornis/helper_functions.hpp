#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <string>

namespace helper_functions
{
inline bool getNthIndex(
  const std::string & string, const char & t, const size_t & n, size_t & index)
{
  size_t count = 0;
  size_t final_find = 0;
  for (uint i = 0; i < string.length(); i++) {
    if (string[i] == t) {
      count++;
      final_find = i;
      if (count == n) {
        index = i;
        return true;
      }
    }
  }
  index = final_find;
  return false;
}
}  // namespace helper_functions

#endif  // HELPER_FUNCTIONS_H_
