#ifndef UI_HELPERS_H_
#define UI_HELPERS_H_

#include <vector>

#include "ncpp/Plane.hh"

namespace ui_helpers
{
inline void sizePlaneToString(ncpp::Plane & plane, const std::string & content)
{
  int row = 1;
  int col = 1;
  int longest_col = 0;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      col++;
      longest_col = col > longest_col ? col : longest_col;
    }
  }
  // If we haven't found an endline char, artificially add a single row, to prevent
  // single line strings from being overwritten by the border. We also add one to the
  // longest col, to account for what would usually be read as the invisible \n
  if (row == 1) {
    row++;
    longest_col++;
  }
  // Add one to longest col to account for border
  plane.resize(row + 2, longest_col + 1);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane.polyfill(row, longest_col, c);
}

inline void sizePlaneToMap(
  ncpp::Plane & plane, const std::string & title,
  const std::map<std::string, std::vector<std::string>> & content_map)
{
  size_t row = 1;
  size_t col = 1;
  size_t longest_col = 0;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const auto & entry : content_map) {
    row++;
    // Check length of map key (Which is used as the divider name)
    longest_col = entry.first.size() > longest_col ? entry.first.size() : longest_col;
    for (const auto & entry_string : entry.second) {
      col = 1;
      for (const char & c : entry_string) {
        if (c == '\n') {
          row++;
          col = 1;
        } else {
          col++;
          longest_col = col > longest_col ? col : longest_col;
        }
      }
      row++;
    }
  }

  // If we haven't found an endline char, artificially add a single row, to prevent
  // single line strings from being overwritten by the border. We also add one to the
  // longest col, to account for what would usually be read as the invisible \n
  if (row == 1) {
    row++;
    longest_col++;
  }

  longest_col = title.size() > longest_col ? title.size() : longest_col;

  // Add one to longest col to account for border
  plane.resize(row + 1, longest_col + 2);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane.polyfill(row, longest_col, c);
}

// Overload to allow for functions to specify how much white space they want around the string
inline void sizePlaneToString(
  ncpp::Plane & plane, const std::string & content, const int & row_buffer, const int & col_buffer)
{
  int row = 1;
  int col = 1;
  int longest_col = 0;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      col++;
      longest_col = col > longest_col ? col : longest_col;
    }
  }
  // If we haven't found an endline char, artificially add a single row, to prevent
  // single line strings from being overwritten by the border. We also add one to the
  // longest col, to account for what would usually be read as the invisible \n
  if (row == 1) {
    row++;
    longest_col++;
  }
  row += row_buffer;
  longest_col += col_buffer;
  // Add one to longest col to account for border
  plane.resize(row + 2, longest_col + 1);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane.polyfill(row, longest_col, c);
}

inline void writeStringToPlane(ncpp::Plane & plane, const std::string & content)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToString(plane, content);

  int row = 1;
  int col = 1;

  ncpp::Cell c(' ');
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
  }

  // FIXME: SHouldn't be doing any styling here
  // uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0, 0, 0);
  // ncchannels_set_bg_alpha(&channel, NCALPHA_TRANSPARENT);

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);
}

// Identical to previous, but renders a cursor at index
inline void writeStringToPlane(
  ncpp::Plane & plane, const std::string & content, const int & cursor_index)
{
  // Cursor index is a location in string. If -1, place at end of string
  plane.erase();
  plane.move_top();
  // As we have a cursor (And therefore expect to be editing the string), we add a horizontal buffer
  ui_helpers::sizePlaneToString(plane, content, 0, 2);

  int string_index = 0;
  int row = 1;
  int col = 1;

  ncpp::Cell c(' ');
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
    if (string_index == cursor_index) {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, "┃");
      nccell_release(plane.to_ncplane(), &cell);
    }
    string_index++;
  }

  // FIXME: SHouldn't be doing any styling here
  // uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0, 0, 0);
  // ncchannels_set_bg_alpha(&channel, NCALPHA_TRANSPARENT);

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);
}

inline void writeStringToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const std::string & content)
{
  plane.erase();
  plane.move_top();

  // Add some white space to the front and end of the title string.
  title.size() > content.size() ? ui_helpers::sizePlaneToString(plane, title)
                                : ui_helpers::sizePlaneToString(plane, content);

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  ncpp::Cell c(' ');
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
  }

  uint64_t channel = plane.get_channels();
  // ncchannels_set_bg_alpha(&channel, NCALPHA_TRANSPARENT);
  plane.perimeter_rounded(0, channel, 0);

  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char & c : title) {
    nccell_load(plane.to_ncplane(), &cell, &c);
    plane.putc(0, col, c);
    nccell_release(plane.to_ncplane(), &cell);
    col++;
  }
}

// Identical to previous, but renders a cursor at index
inline void writeStringToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const std::string & content,
  const int & cursor_index)
{
  plane.erase();
  plane.move_top();

  // Add some white space to the front and end of the title string.
  title.size() > content.size() ? ui_helpers::sizePlaneToString(plane, title, 0, 2)
                                : ui_helpers::sizePlaneToString(plane, content, 0, 2);

  int row = 1;
  int col = 1;
  int string_index = 0;

  // TODO These cell writing loops should be moved to their own functions
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char & c : content) {
    if (c == '\n') {
      row++;
      col = 1;
    } else {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
    if (string_index == cursor_index) {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, "┃");
      nccell_release(plane.to_ncplane(), &cell);
    }
    string_index++;
  }

  // uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0, 0, 0);
  // ncchannels_set_bg_alpha(&channel, NCALPHA_TRANSPARENT);

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char & c : title) {
    nccell_load(plane.to_ncplane(), &cell, &c);
    plane.putc(0, col, c);
    nccell_release(plane.to_ncplane(), &cell);
    col++;
  }
}

inline void writeMapToTitledPlane(
  ncpp::Plane & plane, const std::string & title,
  const std::map<std::string, std::vector<std::string>> & content)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToMap(plane, title, content);

  const int bar_length = plane.get_dim_x();
  const auto fill_char = "─";

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  // ncpp::Cell cell;
  for (const auto & entry : content) {
    // Use map key as divider title
    for (int i = 0; i < bar_length; i++) {
      plane.putstr(row, i, fill_char);
    }
    col = bar_length / 2 - entry.first.size() / 2;
    for (const char & c : entry.first) {
      // plane.get_at(row, col, &cell);
      // cell.
      // nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      // plane.release(cell);
      // nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
    col = 1;
    row++;
    for (const auto & content : entry.second) {
      for (const char & c : content) {
        if (c == '\n') {
          row++;
          col = 1;
        } else {
          // nccell_load(plane.to_ncplane(), &cell, &c);
          plane.putc(row, col, c);
          // nccell_release(plane.to_ncplane(), &cell);
          col++;
        }
      }
      row++;
      col = 1;
    }
  }

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char & c : title) {
    plane.putc(0, col, c);
    col++;
  }
}

// Draws a bar of plane's BG color at top of plane, spanning entire width, with content string centered
inline void drawHelperBar(ncpp::Plane * plane, const std::string content)
{
  const int bar_length = plane->get_dim_x();
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  const auto fill_char = "─";

  for (int i = 0; i < bar_length; i++) {
    plane->putstr(0, i, fill_char);
  }

  int col = bar_length / 2 - content.size() / 2;
  for (const auto & c : content) {
    nccell_load(plane->to_ncplane(), &cell, &c);
    plane->putc(0, col, c);
    nccell_release(plane->to_ncplane(), &cell);
    col++;
  }
}
}  // namespace ui_helpers

#endif  // UI_HELPERS_H_
