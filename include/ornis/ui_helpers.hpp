#ifndef UI_HELPERS_H_
#define UI_HELPERS_H_

#include <functional>
#include <vector>

#include "ncpp/Plane.hh"
#include "ornis/msg_tree.hpp"

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

inline void sizeTree(const msg_tree::MsgTreeNode & node, size_t & rows, size_t & longest_col)
{
  const auto t = &node.getValue();
  const std::string node_line =
    "(" + t->data_type_ + ")  " + t->entry_name_ + ": " + t->entry_data_ + "  ";
  const size_t child_length = node_line.length();
  longest_col = child_length > longest_col ? child_length : longest_col;
  for (const auto & child : node.getChildren()) {
    rows++;
    sizeTree(child, rows, longest_col);
  }
}

inline void sizePlaneToTree(
  ncpp::Plane & plane, const std::string & title, const msg_tree::MsgTree & tree, const bool skip_root = true)
{
  size_t rows = 1;
  size_t longest_col = 0;

  sizeTree(*tree.getRoot(), rows, longest_col);

  longest_col = title.size() > longest_col ? title.size() : longest_col;

  // Add one to longest col to account for border
  const uint extra_rows = skip_root ? 1 : 2;
  plane.resize(rows + extra_rows, longest_col + 2);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane.polyfill(rows, longest_col, c);
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

inline void writeStringVectorToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const std::vector<std::string> & content)
{
  plane.erase();
  plane.move_top();

  const size_t row_count = content.size();

  const auto longest_entry = std::max_element(
    content.begin(), content.end(),
    [](const auto & a, const auto & b) { return a.size() < b.size(); });

  title.size() > longest_entry->size() ? ui_helpers::sizePlaneToString(plane, title)
                                       : ui_helpers::sizePlaneToString(plane, *longest_entry);

  const uint col_count = title.size() > longest_entry->size() ? title.size() : longest_entry->size();
  // Add one to longest col to account for border
  plane.resize(
    row_count + 2, col_count + 2);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(32, 51, 70);
  c.set_fg_rgb8(32, 51, 70);
  plane.polyfill(row_count, col_count, c);

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const std::string & line : content) {
    for (const char & c : line) {
        nccell_load(plane.to_ncplane(), &cell, &c);
        plane.putc(row, col, c);
        nccell_release(plane.to_ncplane(), &cell);
        col++;

    }
    row++;
    col = 1;
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
  std::map<std::string, std::vector<std::string>> & content)
{
  plane.erase();
  plane.move_top();

  // Remove first line in string, which when created from the msg_tree, simply contains the name
  // of the root node
  for (auto & entry : content) {
    for (std::string & content : entry.second) {
      content.erase(0, content.find("\n") + 1);
    }
  }

  ui_helpers::sizePlaneToMap(plane, title, content);

  const int bar_length = plane.get_dim_x();
  const auto fill_char = "─";

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  for (auto & entry : content) {
    for (int i = 0; i < bar_length; i++) {
      plane.putstr(row, i, fill_char);
    }
    col = bar_length / 2 - entry.first.size() / 2;
    for (const char & c : entry.first) {
      plane.putc(row, col, c);
      col++;
    }
    col = 1;
    row++;
    for (std::string & content : entry.second) {
      for (const char & c : content) {
        if (c == '\n') {
          row++;
          col = 1;
        } else {
          plane.putc(row, col, c);
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

// Almost identical to writeDetailedTree, but draws a cursor, as well as a different background color for editable fields
inline void writeEditingTreeToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const msg_tree::MsgTree & tree)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree);

  size_t row = 1;
  int col = 1;

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  std::function<void(
    const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row, const bool & is_root)>
    drawTreeToPlane;
  drawTreeToPlane = [&](
                      const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row,
                      const bool & is_root) {
    // Skip root
    if (!is_root) {
      size_t col = 1;
      const msg_tree::msg_contents t = node.getValue();
      std::string node_line;
      if (node.isEditable()) {
        node_line = "(" + t.data_type_ + ") " + t.entry_name_ + ": " + t.entry_data_;
      } else {
        node_line = t.entry_name_;
      }
      for (const char & c : node_line) {
        plane.putc(row, col, c);
        col++;
      }
      if (node.getEditingStatus()) {
        plane.putc(row, col, "┃");
      }
      if (node.isEditable()) {
        uint64_t highlight = 0;
        ncchannels_set_fg_rgb8(&highlight, 0xff, 0xff, 0xff);
        ncchannels_set_bg_rgb8(&highlight, 72, 91, 120);
        ncchannels_set_bg_alpha(&highlight, ncpp::Cell::AlphaOpaque);
        plane.stain(
          row, col - t.entry_data_.size(), 1, t.entry_data_.size() + 1, highlight, highlight,
          highlight, highlight);
      }
      row++;
    }
    for (const auto & child : node.getChildren()) {
      drawTreeToPlane(child, plane, row, false);
    }
    return;
  };

  drawTreeToPlane(*tree.getRoot(), plane, row, true);

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char & c : title) {
    plane.putc(0, col, c);
    col++;
  }
}

inline void writeSelectionTreeToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const msg_tree::MsgTree & tree,
  const uint & selected_index)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree, false);

  size_t row = 1;
  int col = 1;

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  std::function<void(
    const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row, const bool & is_root,
    bool highlight, uint depth)>
    drawTreeToPlane;
  drawTreeToPlane = [&](
                      const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row,
                      const bool & is_root, bool highlight, uint depth) {
    if (row == selected_index) highlight = true;
    {
      size_t col = 1;
      const msg_tree::msg_contents t = node.getValue();
      std::string node_line;
      node_line.insert(0, depth, ' ');
      if (node.isLeaf()) {
        node_line += "|--" + t.data_type_ + "] " + t.entry_name_;
      } else {
        node_line += t.entry_name_;
      }
      // (Unforuanately) Need to ensure we re-set the highlight channel each draw
      uint64_t highlight_channel;
      if (highlight) {
        ncchannels_set_bg_rgb8(&highlight_channel, 0xff, 0xff, 0xff);
        ncchannels_set_fg_rgb8(&highlight_channel, 72, 91, 120);
      } else {
        ncchannels_set_fg_rgb8(&highlight_channel, 0xff, 0xff, 0xff);
        ncchannels_set_bg_rgb8(&highlight_channel, 72, 91, 120);
      }
      ncchannels_set_bg_alpha(&highlight_channel, ncpp::Cell::AlphaOpaque);

      for (const auto & c : node_line) {
        plane.putc(row, col, c);
        col++;
      }
      plane.stain(
        row, col - node_line.size(), 1, node_line.size() + 1, highlight_channel, highlight_channel,
        highlight_channel, highlight_channel);

      row++;
    }
    for (const auto & child : node.getChildren()) {
      drawTreeToPlane(child, plane, row, false, highlight, depth + 1);
    }
    return;
  };

  uint depth = 0;
  drawTreeToPlane(*tree.getRoot(), plane, row, true, selected_index == 0, depth);

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char & c : title) {
    plane.putc(0, col, c);
    col++;
  }
}

inline void writeDetailedTreeToTitledPlane(
  ncpp::Plane & plane, const std::string & title, const msg_tree::MsgTree & tree)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree);

  size_t row = 1;
  int col = 1;

  std::function<void(
    const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row, const bool & is_root)>
    drawTreeToPlane;
  drawTreeToPlane = [&](
                      const msg_tree::MsgTreeNode & node, ncpp::Plane & plane, size_t & row,
                      const bool & is_root) {
    // Skip root
    if (!is_root) {
      size_t col = 1;
      const msg_tree::msg_contents t = node.getValue();
      std::string node_line = "(" + t.data_type_ + ")  " + t.entry_name_;
      if (!t.entry_data_.empty())
      {
        node_line.append(": " +  t.entry_data_);
      }
      for (const char & c : node_line) {
        plane.putc(row, col, c);
        col++;
      }
      row++;
    }
    for (const auto & child : node.getChildren()) {
      drawTreeToPlane(child, plane, row, false);
    }

    return;
  };

  drawTreeToPlane(*tree.getRoot(), plane, row, true);

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

inline void drawVertLine(
  ncpp::Plane * plane, const uint64_t & p1, const uint64_t & p2, const uint64_t & horz_location,
  const char * symbol)
{
  const int direction = p1 < p2 ? 1 : -1;
  for (uint64_t index = p1; index != p2; index += direction) {
    plane->putc(index, horz_location, symbol);
  }
  plane->putc(p2, horz_location, symbol);
}

inline bool downInput(const ncinput & input)
{
  return (input.id == NCKEY_TAB || input.id == NCKEY_DOWN);
}

inline bool upInput(const ncinput & input)
{
  return ((input.id == NCKEY_TAB && input.shift) || input.id == NCKEY_UP);
}

}  // namespace ui_helpers

#endif  // UI_HELPERS_H_
