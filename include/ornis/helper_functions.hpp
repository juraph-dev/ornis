#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <iostream>
#include <string>
#include <vector>

namespace helper_functions
{
inline bool getNthIndex(
  const std::string & string, const char & target_char, const size_t & target_count, size_t & index)
{
  size_t count = 0;
  size_t final_find = 0;
  for (uint i = 0; i < string.length(); i++) {
    if (string[i] == target_char) {
      count++;
      final_find = i;
      if (count == target_count) {
        index = i;
        return true;
      }
    }
  }
  index = final_find;
  return false;
}

inline void getAllIndexes(
  const std::string & string, const char & target_char, std::vector<size_t> & index)
{
  for (uint i = 0; i < string.length(); i++) {
    if (string[i] == target_char) {
      index.push_back(i);
    }
  }
}

inline bool getDataFromRequestString(
  std::vector<std::string> & data_strings, const std::string & request_string)
{
  std::vector<size_t> data_start_char_indexes, data_end_char_indexes;
  getAllIndexes(request_string, ':', data_start_char_indexes);
  getAllIndexes(request_string, '\n', data_end_char_indexes);

  size_t end_index = 0;
  for (size_t start_index = 0; start_index < data_start_char_indexes.size(); ++start_index) {
    // Get first index of data_end that is greater than data_end_char
    while (true) {
      // If the end char (Usually a newline) occurs before the data start, it doens't belong to the data, so discard
      if (data_end_char_indexes[end_index] < data_start_char_indexes[start_index]) {
        end_index++;
      } else {
        data_strings.push_back(request_string.substr(
          data_start_char_indexes[start_index] + 1, data_end_char_indexes[end_index] - data_start_char_indexes[start_index] - 1));
        end_index++;
        break;
      }
    }
  }
  return true;
}
}  // namespace helper_functions

#endif  // HELPER_FUNCTIONS_H_
