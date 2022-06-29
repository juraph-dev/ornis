#ifndef MSG_TREE_H_
#define MSG_TREE_H_

#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

struct msg_contents
{
  // Type of entry (Double, String, Array etc)
  const std::string data_type_;

  const std::string entry_name_;
  // Contents of entry. Non-const as user will fill out sometimes
  std::string entry_data_;

  bool operator==(msg_contents const & rhs) const
  {
    return this->data_type_ == rhs.data_type_ && this->entry_name_ == rhs.entry_name_;
  }
};

// OStream operator for the msg_contents struct
std::ostream & operator<<(std::ostream & os, const msg_contents & msg)
{
  os << "(" << msg.data_type_ << ") " << msg.entry_name_ << ": " << msg.entry_data_;
  return os;
}

class MsgTreeNode
{
public:
  MsgTreeNode() {}

  MsgTreeNode(const msg_contents & new_msg) : msg_contents_(new_msg) {}

  virtual ~MsgTreeNode() {}

  void addChild(const msg_contents & t) { this->children.push_back(MsgTreeNode(t)); }

  void setValue(const std::string & entry_data) { this->msg_contents_.entry_data_ = entry_data; }

  msg_contents & getValue() { return this->msg_contents_; }

  const msg_contents & getValue() const { return this->msg_contents_; }

  std::vector<MsgTreeNode> & getChildren() { return this->children; }

  const std::vector<MsgTreeNode> & getChildren() const { return this->children; }

  const MsgTreeNode & getChild(size_t index) const { return this->children[index]; }

  MsgTreeNode & getChild(size_t index) { return this->children[index]; }

  bool isLeaf() const { return children.empty(); }

  // the type has to have an overloaded std::ostream << operator for print to work
  void print(const int depth = 0) const
  {
    for (int i = 0; i < depth; ++i) {
      if (i != depth - 1)
        std::cout << "    ";
      else
        std::cout << "|-- ";
    }
    std::cout << this->msg_contents_ << std::endl;
    for (uint i = 0; i < this->children.size(); ++i) {
      this->children.at(i).print(depth + 1);
    }
  }

private:
  msg_contents msg_contents_;
  std::vector<MsgTreeNode> children;
};

class MsgTree
{
public:
  MsgTree(const msg_contents & msg_contents_) : base_(new MsgTreeNode(msg_contents_)) {}
  ~MsgTree();

  private:
    std::unique_ptr<MsgTreeNode> base_;
};

#endif  // MSG_TREE_H_
