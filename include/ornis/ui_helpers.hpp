#ifndef UI_HELPERS_H_
#define UI_HELPERS_H_

#include <algorithm>
#include <codecvt>
#include <cstring>
#include <functional>
#include <locale>
#include <vector>

#include "ncpp/Palette.hh"
#include "ncpp/Plane.hh"
#include "ncpp/Selector.hh"
#include "notcurses/nckeys.h"
#include "notcurses/notcurses.h"
#include "ornis/options.hpp"
#include "ornis/msg_tree.hpp"

namespace ui_helpers
{
inline void sizePlaneToString(ncpp::Plane& plane, const std::string& content, const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);

  int row = 1;
  int col = 1;
  int longest_col = 0;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
      col++;
      longest_col = col > longest_col ? col : longest_col;
    }
  }
  // If we haven't found an endline char, artificially add a single row, to prevent
  // single line strings from being overwritten by the border. We also add one to the
  // longest col, to account for what would usually be read as the invisible \n
  if (row == 1)
  {
    row++;
    longest_col++;
  }
  // Add one to longest col to account for border
  plane.resize(row + 2, longest_col + 1);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(bg.r, bg.g, bg.b);
  c.set_fg_rgb8(bg.r, bg.g, bg.b);
  plane.polyfill(row, longest_col, c);
}

inline void sizePlaneToMap(ncpp::Plane& plane, const std::string& title,
                           const std::map<std::string, std::vector<std::string>>& content_map,
                           const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);
  size_t row = 1;
  size_t col = 1;
  size_t longest_col = 0;

  for (const auto& entry : content_map)
  {
    row++;
    // Check length of map key (Which is used as the divider name)
    longest_col = entry.first.size() > longest_col ? entry.first.size() : longest_col;
    for (const auto& entry_string : entry.second)
    {
      col = 1;
      for (const char& c : entry_string)
      {
        if (c == '\n')
        {
          row++;
          col = 1;
        }
        else
        {
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
  if (row == 1)
  {
    row++;
    longest_col++;
  }

  longest_col = title.size() > longest_col ? title.size() : longest_col;

  // Add one to longest col to account for border
  plane.resize(row + 1, longest_col + 2);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(bg.r, bg.g, bg.b);
  c.set_fg_rgb8(bg.r, bg.g, bg.b);
  plane.polyfill(row, longest_col, c);
}

inline void sizeTree(const msg_tree::MsgTreeNode& node, size_t& rows, size_t& longest_col)
{
  const auto t = &node.getValue();
  const std::string node_line = "(" + t->data_type_ + ")  " + t->entry_name_ + ": " + t->entry_data_ + "  ";
  const size_t child_length = node_line.length();
  longest_col = child_length > longest_col ? child_length : longest_col;
  for (const auto& child : node.getChildren())
  {
    rows++;
    sizeTree(child, rows, longest_col);
  }
}

inline void sizePlaneToTree(ncpp::Plane& plane, const std::string& title, const msg_tree::MsgTree& tree,
                            const Options::color_scheme &color_scheme, const bool skip_root = true)
{
  const auto bg = std::get<2>(color_scheme);
  size_t rows = 1;
  size_t longest_col = 0;

  sizeTree(*tree.getRoot(), rows, longest_col);

  longest_col = title.size() > longest_col ? title.size() : longest_col;

  // Add one to longest col to account for border
  const uint extra_rows = skip_root ? 1 : 2;
  plane.resize(rows + extra_rows, longest_col + 4);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(bg.r, bg.g, bg.b);
  c.set_fg_rgb8(bg.r, bg.g, bg.b);
  plane.polyfill(rows, longest_col, c);
}

// Overload to allow for functions to specify how much white space they want around the string
inline void sizePlaneToString(ncpp::Plane& plane, const std::string& content, const int& row_buffer,
                              const int& col_buffer, const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);
  int row = 1;
  int col = 1;
  int longest_col = 0;

  // iterate through string twice, once to find what size
  // to resize the plane to, second to place the characters on the plane.
  // It's ugly, but much more efficient than dynamically resizing the
  // plane as we iterate through the string.
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
      col++;
      longest_col = col > longest_col ? col : longest_col;
    }
  }
  // If we haven't found an endline char, artificially add a single row, to prevent
  // single line strings from being overwritten by the border. We also add one to the
  // longest col, to account for what would usually be read as the invisible \n
  if (row == 1)
  {
    row++;
    longest_col++;
  }
  row += row_buffer;
  longest_col += col_buffer;
  // Add one to longest col to account for border
  plane.resize(row + 2, longest_col + 1);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_bg_rgb8(bg.r, bg.g, bg.b);
  c.set_fg_rgb8(bg.r, bg.g, bg.b);
  plane.polyfill(row, longest_col, c);
}

inline void writeStringToPlane(ncpp::Plane& plane, const std::string& content, const Options::color_scheme &color_scheme)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToString(plane, content, color_scheme);

  int row = 1;
  int col = 1;

  ncpp::Cell c(' ');
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
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
inline void writeStringToPlane(ncpp::Plane& plane, const std::string& content, const int& cursor_index, const Options::color_scheme &color_scheme)
{
  // Cursor index is a location in string. If -1, place at end of string
  plane.erase();
  plane.move_top();
  // As we have a cursor (And therefore expect to be editing the string), we add a horizontal buffer
  ui_helpers::sizePlaneToString(plane, content, 0, 2, color_scheme);

  int string_index = 0;
  int row = 1;
  int col = 1;

  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
    if (string_index == cursor_index)
    {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, "┃");
      nccell_release(plane.to_ncplane(), &cell);
    }
    string_index++;
  }

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);
}

inline void writeStringToTitledPlane(ncpp::Plane& plane, const std::string& title, const std::string& content, const Options::color_scheme &color_scheme)
{
  plane.erase();
  plane.move_top();

  // Add some white space to the front and end of the title string.
  title.size() > content.size() ? ui_helpers::sizePlaneToString(plane, title, color_scheme) :
                                  ui_helpers::sizePlaneToString(plane, content, color_scheme);

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  ncpp::Cell c(' ');
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
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
  for (const char& c : title)
  {
    nccell_load(plane.to_ncplane(), &cell, &c);
    plane.putc(0, col, c);
    nccell_release(plane.to_ncplane(), &cell);
    col++;
  }
}

inline void writeStringVectorToTitledPlane(ncpp::Plane& plane, const std::string& title,
                                           const std::vector<std::string>& content,  const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);

  plane.erase();
  plane.move_top();

  const size_t row_count = content.size();

  const auto longest_entry = std::max_element(content.begin(), content.end(),
                                              [](const auto& a, const auto& b) { return a.size() < b.size(); });

  title.size() > longest_entry->size() ? ui_helpers::sizePlaneToString(plane, title, color_scheme) :
                                         ui_helpers::sizePlaneToString(plane, *longest_entry, color_scheme);

  const uint col_count = title.size() > longest_entry->size() ? title.size() : longest_entry->size();
  // Add one to longest col to account for border
  plane.resize(row_count + 2, col_count + 2);

  // Fill plane, ensures we don't have a transparent background
  ncpp::Cell c(' ');
  c.set_fg_rgb8(bg.r, bg.g, bg.b);
  c.set_bg_rgb8(bg.r, bg.g, bg.b);
  plane.polyfill(row_count, col_count, c);

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const std::string& line : content)
  {
    for (const char& c : line)
    {
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
  for (const char& c : title)
  {
    nccell_load(plane.to_ncplane(), &cell, &c);
    plane.putc(0, col, c);
    nccell_release(plane.to_ncplane(), &cell);
    col++;
  }
}

// Identical to previous, but renders a cursor at index
inline void writeStringToTitledPlane(ncpp::Plane& plane, const std::string& title, const std::string& content,
                                     const int& cursor_index, const Options::color_scheme &color_scheme)
{
  plane.erase();
  plane.move_top();

  // Add some white space to the front and end of the title string.
  title.size() > content.size() ? ui_helpers::sizePlaneToString(plane, title, 0, 2, color_scheme) :
                                  ui_helpers::sizePlaneToString(plane, content, 0, 2, color_scheme);

  int row = 1;
  int col = 1;
  int string_index = 0;

  // TODO These cell writing loops should be moved to their own functions
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  for (const char& c : content)
  {
    if (c == '\n')
    {
      row++;
      col = 1;
    }
    else
    {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, c);
      nccell_release(plane.to_ncplane(), &cell);
      col++;
    }
    if (string_index == cursor_index)
    {
      nccell_load(plane.to_ncplane(), &cell, &c);
      plane.putc(row, col, "┃");
      nccell_release(plane.to_ncplane(), &cell);
    }
    string_index++;
  }

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char& c : title)
  {
    nccell_load(plane.to_ncplane(), &cell, &c);
    plane.putc(0, col, c);
    nccell_release(plane.to_ncplane(), &cell);
    col++;
  }
}

inline void writeMapToTitledPlane(ncpp::Plane& plane, const std::string& title,
                                  std::map<std::string, std::vector<std::string>>& content, const Options::color_scheme &color_scheme)
{
  plane.erase();
  plane.move_top();

  // Remove first line in string, which when created from the msg_tree, simply contains the name
  // of the root node
  for (auto& entry : content)
  {
    for (std::string& content : entry.second)
    {
      content.erase(0, content.find("\n") + 1);
    }
  }

  ui_helpers::sizePlaneToMap(plane, title, content, color_scheme);

  const int bar_length = plane.get_dim_x();
  const auto fill_char = "─";

  int row = 1;
  int col = 1;

  // TODO These cell writing loops should be moved to their own functions
  for (auto& entry : content)
  {
    for (int i = 0; i < bar_length; i++)
    {
      plane.putstr(row, i, fill_char);
    }
    col = bar_length / 2 - entry.first.size() / 2;
    for (const char& c : entry.first)
    {
      plane.putc(row, col, c);
      col++;
    }
    col = 1;
    row++;
    for (std::string& content : entry.second)
    {
      for (const char& c : content)
      {
        if (c == '\n')
        {
          row++;
          col = 1;
        }
        else
        {
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
  for (const char& c : title)
  {
    plane.putc(0, col, c);
    col++;
  }
}

// Almost identical to writeDetailedTree, but draws a cursor, as well as a different background color for editable
// fields
inline void writeEditingTreeToTitledPlane(ncpp::Plane& plane, const std::string& title, const msg_tree::MsgTree& tree, const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);
  const auto fg = std::get<1>(color_scheme);

  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree, color_scheme);

  size_t row = 1;
  int col = 1;

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  std::function<void(const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, const bool& is_root,
                     uint depth, const std::wstring& prefix)>
      drawTreeToPlane;
  drawTreeToPlane = [&](const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, const bool& is_root,
                        uint depth, const std::wstring& prefix) {
    // Skip root
    if (!is_root)
    {
      size_t col = 1;
      const msg_tree::msg_contents t = node.getValue();

      std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
      std::wstring node_line;
      if (node.isEditable())
      {
        node_line += L'[' + converter.from_bytes(t.data_type_) + L"] " + converter.from_bytes(t.entry_name_) + L": " +
                     converter.from_bytes(t.entry_data_);
      }
      else
      {
        node_line += converter.from_bytes(t.entry_name_);
      }
      node_line.insert(0, prefix);
      // Adjust prefix based on node status
      if (prefix[prefix.length() - 1] == L'│')
      {
        node_line[prefix.length() - 1] = L'├';
      }
      else if (prefix[prefix.length() - 1] == L' ')
      {
        node_line[prefix.length() - 1] = L'└';
      }

      for (const auto& c : node_line)
      {
        plane.putwch(row, col, c);
        col++;
      }
      if (node.getEditingStatus())
      {
        plane.putc(row, col, "┃");
      }
      if (node.isEditable())
      {
        uint64_t highlight = 0;
        ncchannels_set_fg_rgb8(&highlight, fg.r, fg.g, fg.b);
        ncchannels_set_bg_rgb8(&highlight, bg.r, bg.g, bg.b);
        ncchannels_set_bg_alpha(&highlight, ncpp::Cell::AlphaOpaque);
        plane.stain(row, col - t.entry_data_.size(), 1, t.entry_data_.size() + 1, highlight, highlight, highlight,
                    highlight);
      }
      row++;
    }

    for (const auto& child : node.getChildren())
    {
      if (&child == &node.getChildren().back() && child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, false, depth + 2, prefix + L" └");
      }
      else if (&child == &node.getChildren().back())
      {
        drawTreeToPlane(child, plane, row, false, depth + 2, prefix + L"  ");
      }
      else if (child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, false, depth + 2, prefix + L" ├");
      }
      else
      {
        drawTreeToPlane(child, plane, row, false, depth + 2, prefix + L" │");
      }
    }
    return;
  };

  uint depth = 0;
  drawTreeToPlane(*tree.getRoot(), plane, row, true, depth, L"");

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char& c : title)
  {
    plane.putc(0, col, c);
    col++;
  }
}

inline void writeSelectionTreeToTitledPlane(ncpp::Plane& plane, const std::string& title, const msg_tree::MsgTree& tree,
                                            const uint& selected_index, const Options::color_scheme &color_scheme)
{
  const auto bg = std::get<2>(color_scheme);
  const auto fg = std::get<1>(color_scheme);
  const auto hl = std::get<3>(color_scheme);

  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree, color_scheme, false);

  size_t row = 1;
  int col = 1;

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  std::function<void(const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, bool highlight, uint depth,
                     const std::wstring& prefix)>
      drawTreeToPlane;
  drawTreeToPlane = [&](const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, bool highlight, uint depth,
                        const std::wstring& prefix) {
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    if (row == selected_index)
    {
      highlight = true;
    }

    size_t col = 1;
    const msg_tree::msg_contents t = node.getValue();
    std::wstring node_line;
    if (node.isLeaf())
    {
      node_line += L'[' + converter.from_bytes(t.data_type_) + L"]: " + converter.from_bytes(t.entry_name_);
    }
    else
    {
      node_line += converter.from_bytes(t.entry_name_);
    }
    node_line.insert(0, prefix);
    // Adjust prefix based on node status
    if (prefix[prefix.length() - 1] == L'│')
    {
      node_line[prefix.length() - 1] = L'├';
    }
    else if (prefix[prefix.length() - 1] == L' ')
    {
      node_line[prefix.length() - 1] = L'└';
    }

    // (Unfortuanately) Need to ensure we re-set the highlight channel each draw
    uint64_t highlight_channel = 0;
;
    if (highlight)
    {
      ncchannels_set_fg_rgb8(&highlight_channel, fg.r, fg.g, fg.b);
      ncchannels_set_bg_rgb8(&highlight_channel, hl.r, hl.g, hl.b);
    }
    else
    {
      ncchannels_set_fg_rgb8(&highlight_channel, fg.r, fg.g, fg.b);
      ncchannels_set_bg_rgb8(&highlight_channel, bg.r, bg.g, bg.b);
    }
    ncchannels_set_bg_alpha(&highlight_channel, ncpp::Cell::AlphaOpaque);

    for (const wchar_t& c : node_line)
    {
      plane.putwch(row, col, c);
      col++;
    }
    plane.stain(row, col - node_line.size(), 1, node_line.size() + 1, highlight_channel, highlight_channel,
                highlight_channel, highlight_channel);
    row++;

    for (const auto& child : node.getChildren())
    {
      if (&child == &node.getChildren().back() && child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, highlight, depth + 2, prefix + L" └");
      }
      else if (&child == &node.getChildren().back())
      {
        drawTreeToPlane(child, plane, row, highlight, depth + 2, prefix + L"  ");
      }
      else if (child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, highlight, depth + 2, prefix + L" ├");
      }
      else
      {
        drawTreeToPlane(child, plane, row, highlight, depth + 2, prefix + L" │");
      }
    }
    return;
  };

  uint depth = 0;
  drawTreeToPlane(*tree.getRoot(), plane, row, selected_index == 0, depth, L"");

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char& c : title)
  {
    plane.putc(0, col, c);
    col++;
  }
}

inline void writeDetailedTreeToTitledPlane(ncpp::Plane& plane, const std::string& title, const msg_tree::MsgTree& tree, const Options::color_scheme &color_scheme)
{
  plane.erase();
  plane.move_top();

  ui_helpers::sizePlaneToTree(plane, title, tree, color_scheme);

  size_t row = 1;
  int col = 1;

  std::function<void(const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, const std::wstring& prefix,
                     const bool& is_root)>
      drawTreeToPlane;
  drawTreeToPlane = [&](const msg_tree::MsgTreeNode& node, ncpp::Plane& plane, size_t& row, const std::wstring& prefix,
                        const bool& is_root) {
    std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;

    if (!is_root)
    {
      size_t col = 1;
      const msg_tree::msg_contents t = node.getValue();
      std::wstring node_line;
      if (node.isLeaf())
      {
        node_line += L'[' + converter.from_bytes(t.data_type_) + L"]: " + converter.from_bytes(t.entry_name_);
      }
      else
      {
        node_line += converter.from_bytes(t.entry_name_);
      }
      if (!t.entry_data_.empty())
      {
        node_line.append(L": " + converter.from_bytes(t.entry_data_));
      }
      node_line.insert(0, prefix);
      // Adjust prefix based on node status
      if (prefix[prefix.length() - 1] == L'│')
      {
        node_line[prefix.length() - 1] = L'├';
      }
      else if (prefix[prefix.length() - 1] == L' ')
      {
        node_line[prefix.length() - 1] = L'└';
      }

      for (const wchar_t& c : node_line)
      {
        plane.putwch(row, col, c);
        col++;
      }
      row++;
    }
    for (const auto& child : node.getChildren())
    {
      if (&child == &node.getChildren().back() && child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, prefix + L" └", false);
      }
      else if (&child == &node.getChildren().back())
      {
        drawTreeToPlane(child, plane, row, prefix + L"  ", false);
      }
      else if (child.isLeaf())
      {
        drawTreeToPlane(child, plane, row, prefix + L" ├", false);
      }
      else
      {
        drawTreeToPlane(child, plane, row, prefix + L" │", false);
      }
    }
    return;
  };

  drawTreeToPlane(*tree.getRoot(), plane, row, L"", true);

  uint64_t channel = plane.get_channels();
  plane.perimeter_rounded(0, channel, 0);

  // Write planes title
  col = (plane.get_dim_x() - title.size()) / 2;
  for (const char& c : title)
  {
    plane.putc(0, col, c);
    col++;
  }
}

// Draws a bar of plane's BG color at top of plane, spanning entire width, with content string centered
inline void drawHelperBar(ncpp::Plane* plane, const std::string content)
{
  const int bar_length = plane->get_dim_x();
  nccell cell = NCCELL_TRIVIAL_INITIALIZER;
  const auto fill_char = "─";

  for (int i = 0; i < bar_length; i++)
  {
    plane->putstr(0, i, fill_char);
  }

  int col = bar_length / 2 - content.size() / 2;
  for (const auto& c : content)
  {
    nccell_load(plane->to_ncplane(), &cell, &c);
    plane->putc(0, col, c);
    nccell_release(plane->to_ncplane(), &cell);
    col++;
  }
}

inline void drawVertLine(ncpp::Plane* plane, const uint64_t& p1, const uint64_t& p2, const uint64_t& horz_location,
                         const char* symbol)
{
  const int direction = p1 < p2 ? 1 : -1;
  for (uint64_t index = p1; index != p2; index += direction)
  {
    plane->putc(index, horz_location, symbol);
  }
  plane->putc(p2, horz_location, symbol);
}

inline bool downInput(const ncinput& input)
{
  return (input.id == NCKEY_TAB || input.id == NCKEY_DOWN);
}

inline bool upInput(const ncinput& input)
{
  return ((input.id == NCKEY_TAB && input.shift) || input.id == NCKEY_UP);
}

inline bool mouseClick(const ncinput& input)
{
  return (input.id == NCKEY_BUTTON1);
}

// Case for confirmation selections
inline bool selectInput(const ncinput& input)
{
  // Kitty terminal will send both a press, and release input,
  // we create a wrapper here to prevent duplicate inputs
  return (input.id == NCKEY_ENTER);
}

inline bool isPress(const ncinput& input)
{
  // Kitty terminal will send both a press, and release input,
  // Add a lil' diddy here to prevent keys from being sent twice
  return (input.evtype != NCTYPE_RELEASE);
}

}  // namespace ui_helpers

#endif  // UI_HELPERS_H_
