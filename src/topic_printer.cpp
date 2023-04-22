#include "ornis/topic_printer.hpp"

#include "ornis/introspection_functions.hpp"
#include "ornis/ui_helpers.hpp"

TopicPrinter::TopicPrinter(ncpp::Plane* plane, uint height, uint width, std::vector<uint32_t> entry_path,
                           const Options::color_scheme theme)
  : TopicVisualiser(plane, height, width, entry_path, theme), longest_string_(0)
{
  plane_->resize(height_, width_);
  const auto& fg = std::get<1>(theme);
  const auto& bg = std::get<2>(theme);
  uint64_t bgchannels = NCCHANNELS_INITIALIZER(fg.r, fg.b, fg.g, bg.r, bg.b, bg.g);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_OPAQUE);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_OPAQUE);
  plane_->set_channels(bgchannels);
  plane->move_top();
}

TopicPrinter::~TopicPrinter()
{
}

void TopicPrinter::renderData(const rosidl_typesupport_introspection_cpp::MessageMembers* members, uint8_t* data)
{
  const rosidl_typesupport_introspection_cpp::MessageMembers* desired_member;
  uint8_t* member_data = nullptr;
  if (!entry_path_.empty())
  {
    rosidl_typesupport_introspection_cpp::MessageMember member;
    introspection::getMessageMember(entry_path_, members, data, member, &member_data);
    desired_member = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(member.members_->data);
  }
  else
  {
    desired_member = members;
    member_data = data;
  }

  char topic_char[100];
  std::string prefix = desired_member->message_namespace_;
  prefix.replace(prefix.find("::"), sizeof("::") - 1, "/");
  prefix += "/";
  const auto prefix_char = prefix.c_str();

  strcpy(topic_char, prefix_char);
  strcat(topic_char, desired_member->message_name_);

  msg_tree::msg_contents topic_contents = { .data_type_ = "", .entry_name_ = "topic", .entry_data_ = "" };

  auto typesupport =
      introspection::getMessageTypeSupport(topic_char, rosidl_typesupport_introspection_cpp::typesupport_identifier);

  msg_tree::MsgTree topic_msg(topic_contents);
  topic_msg.recursivelyCreateTree(topic_msg.getRoot(), typesupport, member_data);
  ui_helpers::writeDetailedTreeToTitledPlane(*plane_, topic_char, topic_msg);
}
