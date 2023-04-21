#include "ornis/topic_streamer.hpp"

#include <rmw/qos_profiles.h>
#include <rmw/types.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <utility>

#include "ncpp/Plane.hh"
#include "notcurses/notcurses.h"
#include "ornis/introspection_functions.hpp"
#include "ornis/stream_interface.hpp"
#include "ornis/topic_plotter.hpp"
#include "ornis/topic_printer.hpp"
#include "ornis/topic_string_viewer.hpp"
#include "ornis/ui_helpers.hpp"

using namespace std::chrono_literals;

TopicStreamer::TopicStreamer(const std::string& topic_name, const std::string& topic_entry,
                             const std::string& topic_type, const std::string& entry_type,
                             const std::string& entry_path, std::shared_ptr<StreamChannel>& interface_channel,
                             std::shared_ptr<rcl_node_t> ros_interface_node, rcl_context_t context)
  : topic_name_(topic_name)
  , topic_entry_(topic_entry)
  , topic_type_(topic_type)
  , entry_type_(entry_type)
  , entry_path_(entry_path)
  , ros_interface_node_(std::move(ros_interface_node))
  , context_(context)
{
  stream_open_ = true;
  interface_channel_ = interface_channel;

  thread_ = new std::thread([this]() { initialise(); });
}
TopicStreamer::~TopicStreamer()
{
  stream_open_ = false;
  if (thread_ != nullptr)
  {
    thread_->join();
    delete thread_;
  }
}
void TopicStreamer::closeStream()
{
  interface_channel_->stream_plane_->erase();
  stream_open_.store(false);
  interface_channel_->stream_open_.store(false);
}

void TopicStreamer::callback(rcl_subscription_t& subscription, const rosidl_message_type_support_t* type_support)
{
  const auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(type_support->data);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  uint8_t* request_data = static_cast<uint8_t*>(allocator.allocate(members->size_of_, allocator.state));

  // Initialise the memory that is expected to be used by the message
  members->init_function(request_data, rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_message_info_t info;

  // Grab the waiting message
  auto rc = rcl_take(&subscription, request_data, &info, NULL);

  if (rc == RCL_RET_OK)
  {
    topic_visualiser_->renderData(members, request_data);
  }
  else
  {
    // TODO: Test to make sure this fail string actually writes
    const std::string error = "Failed to read message! Error: " + std::to_string(rc);
    ui_helpers::writeStringToPlane(*interface_channel_->stream_plane_, error);
  }
}

void TopicStreamer::waitUntilUiReady()
{
  while (!interface_channel_->stream_open_.load())
  {
  }
}

void TopicStreamer::initialise()
{
  waitUntilUiReady();

  // FIXME Should take ownership of the stream plane, instead of leaving it in the channel. Once
  // Ornis moves to support multiple streams, this will cause issues.

  // Make the stream plane pretty
  ui_helpers::writeStringToTitledPlane(*interface_channel_->stream_plane_, topic_name_, "Waiting for message");

  const auto type_support = introspection::getMessageTypeSupport(
      topic_type_.c_str(), rosidl_typesupport_introspection_cpp::typesupport_identifier);

  auto* members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(type_support->data);

  std::vector<std::string> entry_path_vec;
  // Split string into std::vector
  size_t last = 0;
  size_t next = 0;
  while ((next = entry_path_.find('/', last)) != std::string::npos)
  {
    const auto t_string = entry_path_.substr(last, next - last);
    if (!t_string.empty())
    {
      entry_path_vec.push_back(t_string);
    }
    last = next + 1;
  }
  entry_path_vec.push_back(entry_path_.substr(last));
  // We also trim out the first entry, as it's simply the message name
  entry_path_vec.erase(entry_path_vec.begin());

  if (!entry_path_vec.empty())
  {
    offset_ = introspection::getEntryOffset(entry_path_vec, entry_type_, members);
  }
  // If we are grabbing the whole message, we obviously have neither an entry path, nor an offset
  // visualise message as a plain string
  if (entry_type_ == "Msg" || entry_type_.empty())
  {
    topic_visualiser_ =
        std::make_unique<TopicPrinter>(TopicPrinter(interface_channel_->stream_plane_.get(), 20, 80, offset_));
  }
  else
  {
    // Requested a single subelement. Attempt to visualise accordingly.
    rosidl_typesupport_introspection_cpp::MessageMember found_member;
    introspection::getMessageMember(offset_, members, found_member);
    if (introspection::parsableAsNumeric(found_member))
    {
      topic_visualiser_ =
          std::make_unique<TopicPlotter>(TopicPlotter(interface_channel_->stream_plane_.get(), 20, 80, offset_));
    }
    else
    {
      topic_visualiser_ = std::make_unique<TopicStringViewer>(
          TopicStringViewer(interface_channel_->stream_plane_.get(), 20, 80, offset_));
    }
  }

  // TODO: Investigate swapping profiles at runtime
  // There are some get_qos functions available. Investigate.
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();

  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  subscription_options.qos = qos_profile;

  auto ret = rcl_subscription_init(&subscription, ros_interface_node_.get(), type_support, topic_name_.c_str(),
                                   &subscription_options);

  ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context_, rcl_get_default_allocator());
  size_t index;

  while (stream_open_.load())
  {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_wait_set_add_subscription(&wait_set, &subscription, &index);
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(10000));

    if (ret == RCL_RET_TIMEOUT)
    {
      ui_helpers::writeStringToTitledPlane(*interface_channel_->stream_plane_, topic_name_,
                                           "Timed out waiting for message!");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    if (wait_set.subscriptions[0])
    {
      callback(subscription, type_support);
    }
  }

  // Clean up
  ret = rcl_subscription_fini(&subscription, ros_interface_node_.get());
  ret = rcl_wait_set_fini(&wait_set);

  interface_channel_->stream_plane_->erase();
  interface_channel_->stream_plane_.reset();
}
