#ifndef MSG_TREE_H_
#define MSG_TREE_H_

#include <rosidl_runtime_c/message_type_support_struct.h>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include "ornis/introspection_functions.hpp"
#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace msg_tree
{
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
  // OStream operator for the msg_contents struct
  friend std::ostream & operator<<(std::ostream & os, const msg_contents & msg)
  {
    os << "(" << msg.data_type_ << ") " << msg.entry_name_ << ": " << msg.entry_data_;
    return os;
  }
};

class MsgTreeNode
{
public:
  MsgTreeNode() {}

  MsgTreeNode(const msg_contents & new_msg, const MsgTreeNode * parent)
  : msg_contents_(new_msg), parent_(parent)
  {
  }

  virtual ~MsgTreeNode() {}

  MsgTreeNode * addChild(const msg_contents & t)
  {
    this->children_.push_back(MsgTreeNode(t, this));
    return &this->children_.back();
  }

  void setValue(const std::string & entry_data) { this->msg_contents_.entry_data_ = entry_data; }

  msg_contents & getValue() { return this->msg_contents_; }

  const msg_contents & getValue() const { return this->msg_contents_; }

  std::vector<MsgTreeNode> & getChildren() { return this->children_; }

  const std::vector<MsgTreeNode> & getChildren() const { return this->children_; }

  const MsgTreeNode & getChild(size_t index) const { return this->children_[index]; }

  MsgTreeNode & getChild(size_t index) { return this->children_[index]; }

  const MsgTreeNode * getParent() { return parent_; }

  bool isLeaf() const { return children_.empty(); }

  void toString(std::string & output, int indent) const
  {
    const std::string indent_str(indent, ' ');
    output.append("\n" + indent_str + this->msg_contents_.entry_name_);

    for (const auto & child : children_) {
      child.toString(output, indent + 2);
    }
  }

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
    for (uint i = 0; i < this->children_.size(); ++i) {
      this->children_.at(i).print(depth + 1);
    }
  }

private:
  msg_contents msg_contents_;
  std::vector<MsgTreeNode> children_;
  const MsgTreeNode * parent_;
};

class MsgTree
{
public:
  MsgTree(const msg_contents & msg_contents_, const rosidl_message_type_support_t * type_data)
  : base_(new MsgTreeNode(msg_contents_, nullptr))
  {
    recursivelyCreateTree(base_.get(), type_data);
  }

  ~MsgTree() {}

  const MsgTreeNode * getRoot() const { return base_.get(); }

  MsgTreeNode * getRoot() { return base_.get(); }

  void recursivelyCreateTree(
    MsgTreeNode * target_node, const rosidl_message_type_support_t * type_data)
  {
    using rosidl_typesupport_introspection_cpp::MessageMember;
    using rosidl_typesupport_introspection_cpp::MessageMembers;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
    using rosidl_typesupport_introspection_cpp::ServiceMembers;

    const auto * members = static_cast<const MessageMembers *>(type_data->data);
    target_node->getChildren().reserve(members->member_count_);
    for (size_t i = 0; i < members->member_count_; i++) {
      const MessageMember & member = members->members_[i];
      std::string new_node_type;
      introspection::messageTypeToString(member, new_node_type);
      const msg_contents msg_data = {.data_type_ = new_node_type, .entry_name_ = member.name_};

      MsgTreeNode * new_node = target_node->addChild(msg_data);

      if (member.is_array_) {
        const msg_contents msg_array_data = {.data_type_ = "Array", .entry_name_ = "[]"};
        new_node->getChildren().reserve(1);
        new_node = new_node->addChild(msg_array_data);
      }
      if (member.type_id_ == ROS_TYPE_MESSAGE) {
        recursivelyCreateTree(new_node, member.members_);
      }
    }
  }

private:
  std::unique_ptr<MsgTreeNode> base_;
};

}  // namespace msg_tree
#endif  // MSG_TREE_H_
