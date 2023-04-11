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

  // OStream operator
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

  void setEditingStatus(const bool & edit_status) { being_edited_ = edit_status; }

  bool getEditingStatus() const { return being_edited_; }

  msg_contents & getValue() { return this->msg_contents_; }

  const msg_contents & getValue() const { return this->msg_contents_; }

  std::vector<MsgTreeNode> & getChildren() { return this->children_; }

  const std::vector<MsgTreeNode> & getChildren() const { return this->children_; }

  bool isEditable() const { return this->children_.size() == 0; }

  const MsgTreeNode & getChild(size_t index) const { return this->children_[index]; }

  MsgTreeNode & getChild(size_t index) { return this->children_[index]; }

  const MsgTreeNode * getParent() { return parent_; }

  bool isLeaf() const { return children_.empty(); }

  size_t leafCount() const { return children_.size(); }

  void toString(std::string & output, int indent = 0) const
  {
    const std::string indent_str(indent, ' ');
    if (indent != 0) {
      output.append("\n");
    }
    output.append(indent_str + this->msg_contents_.entry_name_);

    for (const auto & child : children_) {
      child.toString(output, indent + 2);
    }
  }

  void print(const int depth = 0) const
  {
    for (int i = 0; i < depth; ++i) {
      if (i != depth - 1)
        std::cout << "    ";
      else
        std::cout << "╰── ";
    }
    std::cout << this->msg_contents_ << std::endl;
    for (uint i = 0; i < this->children_.size(); ++i) {
      this->children_.at(i).print(depth + 1);
    }
  }

  MsgTreeNode * getNthEditableNode(const uint & n)
  {
    uint search_index = 1;
    return findNthEditableNode(n, search_index);
  }

  MsgTreeNode * findNthEditableNode(const uint & n, uint & search_index)
  {
    MsgTreeNode * editable_leaf = nullptr;
    if (isEditable()) {
      if (search_index == n) {
        return this;
      } else {
        search_index++;
      }
    } else {
      for (auto & child : children_) {
        editable_leaf = child.findNthEditableNode(n, search_index);
        if (editable_leaf != nullptr) {
          return editable_leaf;
        }
      }
    }
    return editable_leaf;
  }

  std::string getPathNthNode(const uint & n)
  {
    uint search_index = 1;
    return findPathNthNode(n, search_index);
  }

  std::string findPathNthNode(const uint & n, uint & search_index)
  {
    std::string path = msg_contents_.entry_name_;
    if (path[0] != '/') {
      path.insert(path.begin(), '/');
    }
    if (search_index == n) {
      return path;
    }
    search_index++;
    for (auto & child : children_) {
      const auto subpath = child.findPathNthNode(n, search_index);
      if (!subpath.empty()) {
        path.append(subpath);
        return path;
      }
    }
    return "";
  }

  MsgTreeNode * getNthNode(const uint & n)
  {
    uint search_index = 1;
    return findNthNode(n, search_index);
  }

  MsgTreeNode * findNthNode(const uint & n, uint & search_index)
  {
    MsgTreeNode * leaf = nullptr;
    if (search_index == n) {
      return this;
    }
    search_index++;
    for (auto & child : children_) {
      leaf = child.findNthNode(n, search_index);
      if (leaf != nullptr) {
        return leaf;
      }
    }
    return leaf;
  }

  void writeNodeToMessage(
    uint8_t * message_data, const rosidl_typesupport_introspection_cpp::MessageMembers * members)
  {
    for (size_t i = 0; i < members->member_count_; i++) {
      const rosidl_typesupport_introspection_cpp::MessageMember & member = members->members_[i];
      uint8_t * member_data = &message_data[member.offset_];
      // Perform a check for if we're dealing with a ros message type, and recurse if we are
      if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const auto sub_members =
          static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
            member.members_->data);
        children_[i].writeNodeToMessage(member_data, sub_members);
      } else {
        introspection::stringToMessageData(
          message_data, member, children_[i].msg_contents_.entry_data_);
      }
    }
  }

private:
  msg_contents msg_contents_;
  std::vector<MsgTreeNode> children_;
  const MsgTreeNode * parent_;

  // If the user is editing this node
  // TODO: Rename to "currently_selected" or something similar
  bool being_edited_ = false;
};

class MsgTree
{
public:
  MsgTree(const msg_contents & msg_contents_, const rosidl_message_type_support_t * type_data)
  : node_count_(0), editable_node_count_(0), base_(new MsgTreeNode(msg_contents_, nullptr))
  {
    recursivelyCreateTree(base_.get(), type_data);
  }

  // If constructed with no typesupport, no tree is created. Used for if tree is constructed
  // after root node is created.
  MsgTree(const msg_contents & msg_contents_)
  : node_count_(0), editable_node_count_(0), base_(new MsgTreeNode(msg_contents_, nullptr))
  {
  }

  ~MsgTree() {}

  const MsgTreeNode * getRoot() const { return base_.get(); }
  MsgTreeNode * getRoot() { return base_.get(); }

  const MsgTreeNode * getFirstEditableNode() const { return base_->getNthEditableNode(1); }

  void recursivelyCreateTree(
    MsgTreeNode * target_node, const rosidl_message_type_support_t * type_data)
  {
    using rosidl_typesupport_introspection_cpp::MessageMember;
    using rosidl_typesupport_introspection_cpp::MessageMembers;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
    const auto * members = static_cast<const MessageMembers *>(type_data->data);

    // To prevent potential duplication, Remove Node's children if any exist
    if (!target_node->getChildren().empty()) {
      target_node->getChildren().clear();
    }
    target_node->getChildren().reserve(members->member_count_);

    for (size_t i = 0; i < members->member_count_; i++) {
      const MessageMember & member = members->members_[i];
      std::string new_node_type;
      introspection::messageTypeToString(member, new_node_type);
      const msg_contents msg_data = {
        .data_type_ = new_node_type, .entry_name_ = member.name_, .entry_data_ = ""};

      MsgTreeNode * new_node = target_node->addChild(msg_data);

      if (member.is_array_) {
        const msg_contents msg_array_data = {
          .data_type_ = "Array", .entry_name_ = "[]", .entry_data_ = ""};
        new_node->getChildren().reserve(1);
        new_node->addChild(msg_array_data);
        editable_node_count_++; // An array is editable
        node_count_++;
      }
      else if (member.type_id_ == ROS_TYPE_MESSAGE) {
        recursivelyCreateTree(new_node, member.members_);
      } else {
        // Node has no chldren, probably editable
        editable_node_count_++;
      }
      node_count_++;
    }
  }

  void recursivelyCreateTree(
    MsgTreeNode * target_node, const rosidl_message_type_support_t * type_data,
    uint8_t * message_data)
  {
    using rosidl_typesupport_introspection_cpp::MessageMember;
    using rosidl_typesupport_introspection_cpp::MessageMembers;
    using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
    const auto * members = static_cast<const MessageMembers *>(type_data->data);

    // To prevent potential duplication, Remove Node's children if any exist
    if (!target_node->getChildren().empty()) {
      target_node->getChildren().clear();
    }
    target_node->getChildren().reserve(members->member_count_);

    for (size_t i = 0; i < members->member_count_; i++) {
      const MessageMember & member = members->members_[i];
      std::string new_node_type;
      introspection::messageTypeToString(member, new_node_type);
      const msg_contents msg_data = {
        .data_type_ = new_node_type, .entry_name_ = member.name_, .entry_data_ = ""};

      MsgTreeNode * new_node = target_node->addChild(msg_data);

      if (member.is_array_) {
        const msg_contents msg_array_data = {
          .data_type_ = "Array", .entry_name_ = "[]", .entry_data_ = ""};
        new_node->getChildren().reserve(1);
        new_node->addChild(msg_array_data);
        node_count_++;
        editable_node_count_++; // an array is editable
      }
      if (member.type_id_ == ROS_TYPE_MESSAGE) {
        uint8_t * sub_member_data = &message_data[member.offset_];
        recursivelyCreateTree(new_node, member.members_, sub_member_data);
      } else {
        // Node has no chldren, probably editable
        editable_node_count_++;
        introspection::messageDataToString(
          member, &message_data[member.offset_], new_node->getValue().entry_data_);
      }
      node_count_++;
    }
  }

  void writeTreeToMessage(
    uint8_t * message_data,
    const rosidl_typesupport_introspection_cpp::MessageMembers * members) const
  {
    base_->writeNodeToMessage(message_data, members);
  }

  uint node_count_;
  uint editable_node_count_;

private:
  std::unique_ptr<MsgTreeNode> base_;
};

}  // namespace msg_tree
#endif  // MSG_TREE_H_
