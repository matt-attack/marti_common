// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************
#ifndef SWRI_ROSCPP_SUBSCRIBER_IMPL_H_
#define SWRI_ROSCPP_SUBSCRIBER_IMPL_H_

#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>

#include <swri_roscpp/logging.h>

namespace swri
{

#define DURATION_MAX rclcpp::Duration(1000000, 0)
#define DURATION_MIN rclcpp::Duration(0, 0)
class Subscriber;
class SubscriberImpl
{
 public:
  std::shared_ptr<rclcpp::Node> nh_;
 
 protected:
  rclcpp::SubscriptionBase::SharedPtr sub_;
  std::string unmapped_topic_;
  std::string mapped_topic_;

  int message_count_;

  rclcpp::Time last_header_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  rclcpp::Time last_receive_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  rclcpp::Duration total_latency_ = DURATION_MIN;
  rclcpp::Duration min_latency_ = DURATION_MAX;
  rclcpp::Duration max_latency_ = DURATION_MIN;

  rclcpp::Duration total_periods_ = DURATION_MAX;
  rclcpp::Duration min_period_ = DURATION_MAX;
  rclcpp::Duration max_period_ = DURATION_MIN;

  rclcpp::Duration timeout_ = rclcpp::Duration(0, 0);
  bool in_timeout_;
  int timeout_count_;
  bool blocking_timeout_;

  void processHeader(const rclcpp::Time &stamp)
  {
    rclcpp::Time now = nh_->now();

    // Check for timeouts so that we can correctly increment the
    // timeout count.
    checkTimeout(now);

    message_count_++;

    rclcpp::Duration latency = now - stamp;
    if (message_count_ == 1) {
      min_latency_ = latency;
      max_latency_ = latency;
      total_latency_ = latency;
    } else {
      min_latency_ = std::min(min_latency_, latency);
      max_latency_ = std::max(max_latency_, latency);
      total_latency_ = total_latency_ + latency;
    }

    if (message_count_ > 1) {
      rclcpp::Duration period = now - last_receive_time_;
      if (message_count_ == 2) {
        min_period_ = period;
        max_period_ = period;
        total_periods_ = period;
      } else if (message_count_ > 2) {
        min_period_ = std::min(min_period_, period);
        max_period_ = std::max(max_period_, period);
        total_periods_ = total_periods_ + period;
      }
    }

    // Reset the timeout condition to false.
    in_timeout_ = false;

    last_receive_time_ = now;
    last_header_stamp_ = stamp;
  }

  void checkTimeout(const rclcpp::Time &now)
  {
    if (blocking_timeout_) {
      return;
    }

    if (in_timeout_ || timeout_ <= rclcpp::Duration(0, 0)) {
      return;
    }

    if (message_count_ == 0) {
      return;
    }

    if (age(now) > timeout_) {
      in_timeout_ = true;
      timeout_count_++;
    }
  }


 public:
  SubscriberImpl()
  {
    unmapped_topic_ = "N/A";
    mapped_topic_ = "N/A";
    timeout_ = rclcpp::Duration(-1,0);
    blocking_timeout_ = false;
    resetStatistics();
  }

  const std::string& mappedTopic() const
  {
    return mapped_topic_;
  }

  const std::string& unmappedTopic() const
  {
    return unmapped_topic_;
  }

  int numPublishers() const
  {
    return 0;//sub_.getNumPublishers();
  }

  void resetStatistics()
  {
    message_count_ = 0;
    in_timeout_ = false;
    timeout_count_ = 0;
  }

  int messageCount() const
  {
    return message_count_;
  }

  rclcpp::Duration age(const rclcpp::Time &now) const
  {
    if (message_count_ < 1) {
      return rclcpp::Duration(1000000, 1000000);
    } else {
      return now - last_header_stamp_;
    }
  }

  rclcpp::Duration meanLatency() const
  {
    if (message_count_ < 1) {
      return DURATION_MAX;
    } else {
      return rclcpp::Duration((total_latency_.nanoseconds()/1000000000.0) / message_count_);
    }
  }

  rclcpp::Duration minLatency() const
  {
    if (message_count_ < 1) {
      return DURATION_MAX;
    } else {
      return min_latency_;
    }
  }

  rclcpp::Duration maxLatency() const
  {
    if (message_count_ < 1) {
      return DURATION_MAX;
    } else {
      return max_latency_;
    }
  }

  double meanFrequencyHz() const
  {
    if (message_count_ < 2) {
      return 0.0;
    } else {
      return 1e9 / meanPeriod().nanoseconds();
    }
  }

  rclcpp::Duration meanPeriod() const
  {
    if (message_count_ < 2) {
      return DURATION_MAX;
    } else {
      double seconds = (total_periods_.nanoseconds()/1000000000.0) / (message_count_ - 1);
      return rclcpp::Duration(seconds, static_cast<int32_t>(seconds*1000000000.0)%1000000000);
    }
  }

  rclcpp::Duration minPeriod() const
  {
    if (message_count_ < 2) {
      return DURATION_MAX;
    } else {
      return min_period_;
    }
  }

  rclcpp::Duration maxPeriod() const
  {
    if (message_count_ < 2) {
      return DURATION_MAX;
    } else {
      return max_period_;
    }
  }

  void setTimeout(const rclcpp::Duration &time_out)
  {
    timeout_ = time_out;
    in_timeout_ = false;
    timeout_count_ = 0;
  }

  bool blockTimeouts(bool block) {
    if (block) {
      in_timeout_ = false;
    }

    bool old_block = blocking_timeout_;
    blocking_timeout_ = block;
    return old_block;
  }

  bool timeoutsBlocked() const {
    return blocking_timeout_;
  }

  rclcpp::Duration timeout() const
  {
    return timeout_;
  }

  bool timeoutEnabled() const
  {
    return timeout_ > rclcpp::Duration(0, 0);
  }

  bool inTimeout()
  {
    checkTimeout(nh_->now());
    return in_timeout_;
  }

  int timeoutCount()
  {
    checkTimeout(nh_->now());
    return timeout_count_;
  }
};  // class SubscriberImpl

struct TrueType
{
  static const bool value = true;
};

template<class M , class T>
class TypedSubscriberImpl : public SubscriberImpl
{
  T *obj_;
  void (T::*callback_)(const std::shared_ptr< M  > );

 public:
  TypedSubscriberImpl(
    swri::Node* nh,
    const std::string &topic,
    uint32_t queue_size,
    void(T::*fp)(const std::shared_ptr< M  > ),
    T *obj,
    const rmw_qos_profile_t& transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh->ResolveName(topic);
    nh_ = nh->nh_;

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    callback_ = fp;
    obj_ = obj;
    //transport_hints.depth = queue_size;
    sub_ = nh->nh_->create_subscription<M>(mapped_topic_,
                        std::bind(&TypedSubscriberImpl::handleMessage,
                        this, std::placeholders::_1),
                        transport_hints);
  }

  // Handler for messages with headers
  /*template <class M2>
  typename std::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const std::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    (obj_->*callback_)(msg);
  }*/

  // Handler for messages without headers
  //template <class M2>
  //typename std::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  void handleMessage(const std::shared_ptr< M > msg)
  {
    processHeader(nh_->now());
    (obj_->*callback_)(msg);
  }
};  // class TypedSubscriberImpl

template<class M>
class BindSubscriberImpl : public SubscriberImpl
{
  std::function<void(const std::shared_ptr< M > )> callback_;
  

 public:
  BindSubscriberImpl(
    swri::Node* nh,
    const std::string &topic,
    uint32_t queue_size,
    const std::function<void(const std::shared_ptr< M > )> &callback,
    const rmw_qos_profile_t& transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh->ResolveName(topic);
    nh_ = nh->nh_;

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    callback_ = callback;

    transport_hints.depth = queue_size;
    sub_ = nh->nh_->create_subscription(mapped_topic_,
                        std::bind(&BindSubscriberImpl::handleMessage,
                        this, std::placeholders::_1),
                        transport_hints);
  }

  // Handler for messages with headers
  /*template <class M2>
  typename std::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const std::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    callback_(msg);
  }*/

  // Handler for messages without headers
  //template <class M2>
  //typename std::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  void handleMessage(const std::shared_ptr< M > msg)
  {
    processHeader(nh_->now());
    callback_(msg);
  }
};  // class BindSubscriberImpl

template<class M>
class StorageSubscriberImpl : public SubscriberImpl
{
  std::shared_ptr< M > *dest_;

 public:
  StorageSubscriberImpl(
    swri::Node* nh,
    const std::string &topic,
    std::shared_ptr< M > *dest,
    const rmw_qos_profile_t& transport_hints)
  {
    unmapped_topic_ = topic;
    mapped_topic_ = nh->ResolveName(topic);
    nh_ = nh->nh_;

    if (unmapped_topic_ == mapped_topic_) {
      ROS_INFO("Subscribing to '%s'.", mapped_topic_.c_str());
    } else {
      ROS_INFO("Subscribing to '%s' at '%s'.",
               unmapped_topic_.c_str(),
               mapped_topic_.c_str());
    }

    dest_ = dest;
    //transport_hints.depth = 2;
    sub_ = nh->nh_->create_subscription<M>(mapped_topic_,
                        std::bind(&StorageSubscriberImpl::handleMessage,
                        this, std::placeholders::_1),
                        transport_hints);
  }

  // Handler for messages with headers
  /*template <class M2>
  typename std::enable_if< ros::message_traits::HasHeader<M2>, void>::type
  handleMessage(const std::shared_ptr< M const> &msg)
  {
    processHeader(msg->header.stamp);
    *dest_ = msg;
  }*/

  // Handler for messages without headers
  //template <class M2>
  //typename std::disable_if< ros::message_traits::HasHeader<M2>, void>::type
  void handleMessage(const std::shared_ptr< M > msg)
  {
    processHeader(nh_->now());
    *dest_ = msg;
  }
};  // class StorageSubscriberImpl
}  // namespace swri
#endif  // SWRI_ROSCPP_SUBSCRIBER_IMPL_H_
