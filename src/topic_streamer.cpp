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
#include "ornis/ui_helpers.hpp"

using namespace std::chrono_literals;

TopicStreamer::TopicStreamer(
  const std::string & topic_name, const std::string & topic_type,
  std::shared_ptr<StreamChannel> & interface_channel,
  std::shared_ptr<rcl_node_t> ros_interface_node, rcl_context_t context)
: topic_name_(topic_name),
  topic_type_(topic_type),
  ros_interface_node_(std::move(ros_interface_node)),
  context_(context)
{
  stream_open_ = true;
  interface_channel_ = interface_channel;

  thread_ = new std::thread([this]() { initialise(); });
}
TopicStreamer::~TopicStreamer()
{
  if (thread_ != nullptr) {
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

void TopicStreamer::callback(
  rcl_subscription_t & subscription, const rosidl_message_type_support_t * type_support)
{
  const auto members =
    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);

  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  uint8_t * request_data =
    static_cast<uint8_t *>(allocator.allocate(members->size_of_, allocator.state));

  // Initialise the memory that is expected to be used by the message
  members->init_function(request_data, rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_message_info_t info;

  // Grab the waiting message
  auto rc = rcl_take(&subscription, request_data, &info, NULL);

  if (rc == RCL_RET_OK) {
    topic_visualiser_->renderData(interface_channel_->stream_plane_.get(), members);

    // Add decorations to plane, now that it is the correct size
  } else {
    // TODO: Test to make sure this fail string actually writes
    const std::string error = "Failed to read message! Error: " + std::to_string(rc);
    ui_helpers::writeStringToPlane(*interface_channel_->stream_plane_, error);
  }
}

void TopicStreamer::waitUntilUiReady()
{
  while (!interface_channel_->stream_open_.load()) {
  }
}

void TopicStreamer::initialise()
{
  waitUntilUiReady();

  // FIXME Should take ownership of the stream plane, instead of leaving it in the channel. Once
  // Ornis moves to support multiple streams, this will cause issues.
  // Make the stream plane pretty

  uint64_t bgchannels = NCCHANNELS_INITIALIZER(255, 255, 255, 32, 51, 70);
  ncchannels_set_fg_alpha(&bgchannels, NCALPHA_OPAQUE);
  ncchannels_set_bg_alpha(&bgchannels, NCALPHA_OPAQUE);
  interface_channel_->stream_plane_->set_base("", 0, bgchannels);

  ui_helpers::writeStringToTitledPlane(
    *interface_channel_->stream_plane_, topic_name_, "Waiting for message");

  // TODO: This can be allocated to a variable in header, doesn't NEED to be passed at each callback
  const auto type_support = introspection::getMessageTypeSupport(
    topic_type_.c_str(), rosidl_typesupport_introspection_cpp::typesupport_identifier);

  // Determine how to visualise the message
  // HACK Hardcoded for now
  topic_visualiser_ = std::make_unique<TopicPlotter>(TopicPlotter());

  // TODO: Investigate swapping profiles at runtime
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();

  rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  subscription_options.qos = qos_profile;

  auto ret = rcl_subscription_init(
    &subscription, ros_interface_node_.get(), type_support, topic_name_.c_str(),
    &subscription_options);

  ret = rcl_wait_set_init(&wait_set, 1, 0, 0, 0, 0, 0, &context_, rcl_get_default_allocator());
  size_t index;

  while (stream_open_.load()) {
    ret = rcl_wait_set_clear(&wait_set);
    ret = rcl_wait_set_add_subscription(&wait_set, &subscription, &index);
    ret = rcl_wait(&wait_set, RCL_MS_TO_NS(10000));

    if (ret == RCL_RET_TIMEOUT) {
      ui_helpers::writeStringToTitledPlane(
        *interface_channel_->stream_plane_, topic_name_, "Timed out waiting for message!");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      continue;
    }
    if (wait_set.subscriptions[0]) {
      callback(subscription, type_support);
    }
  }

  // Clean up
  ret = rcl_subscription_fini(&subscription, ros_interface_node_.get());
  ret = rcl_wait_set_fini(&wait_set);

  interface_channel_->stream_plane_->erase();
  interface_channel_->stream_plane_.reset();
}
