#ifndef UI_HELPERS_H_
#define UI_HELPERS_H_

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
  // Add one to longest col to account for boreder
  plane.resize(row + 1, longest_col + 1);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  plane.polyfill(row, longest_col, c);
}

  // Identical to previous, but renders a cursor at index
inline void writeStringToPlane(ncpp::Plane & plane, const std::string & content, const int &cursor_index)
{
  // Cursor index is a location in string. If -1, place at end of string
  plane.erase();
  plane.move_top();
  ui_helpers::sizePlaneToString(plane, content);

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
  uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);
  plane.perimeter_rounded(0, channel, 0);
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
  uint64_t channel = NCCHANNELS_INITIALIZER(0xf0, 0xa0, 0xf0, 0x10, 0x10, 0x60);
  plane.perimeter_rounded(0, channel, 0);
}
}


#endif // UI_HELPERS_H_
