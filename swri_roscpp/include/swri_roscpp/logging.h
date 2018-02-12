// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_LOGGING_H_
#define SWRI_ROSCPP_LOGGING_H_

#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>

#include <sstream>

#define ROS_ERROR(...) RCLCPP_ERROR(swri::get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(swri::get_logger(), __VA_ARGS__)
#define ROS_INFO(...) RCLCPP_INFO(swri::get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(swri::get_logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(swri::get_logger(), __VA_ARGS__)

#define ROS_ERROR_THROTTLE(rate, ...) RCLCPP_ERROR(swri::get_logger(), __VA_ARGS__)
#define ROS_WARN_THROTTLE(rate, ...) RCLCPP_WARN(swri::get_logger(), __VA_ARGS__)
#define ROS_INFO_THROTTLE(rate, ...) RCLCPP_INFO(swri::get_logger(), __VA_ARGS__)
#define ROS_DEBUG_THROTTLE(rate, ...) RCLCPP_INFO(swri::get_logger(), __VA_ARGS__)

#define ROS_ERROR_ONCE(...) RCLCPP_ERROR(swri::get_logger(), __VA_ARGS__)
#define ROS_WARN_ONCE(...) RCLCPP_WARN(swri::get_logger(), __VA_ARGS__)
#define ROS_INFO_ONCE(...) RCLCPP_INFO(swri::get_logger(), __VA_ARGS__)
#define ROS_DEBUG_ONCE(...) RCLCPP_INFO(swri::get_logger(), __VA_ARGS__)

#define ROS_INFO_STREAM(args) \
  do { \
    std::stringstream s; \
    s << args; \
    RCLCPP_INFO(swri::get_logger(), "%s", s.str().c_str()); \
  } \
  while (0)

#define ROS_DEBUG_STREAM(args) \
  do { \
    std::stringstream s; \
    s << args; \
    RCLCPP_DEBUG(swri::get_logger(), "%s", s.str().c_str()); \
  } \
  while (0)

#define ROS_WARN_STREAM(args) \
  do { \
    std::stringstream s; \
    s << args; \
    RCLCPP_WARN(swri::get_logger(), "%s", s.str().c_str()); \
  } \
  while (0)

#define ROS_ERROR_STREAM(args) \
  do { \
    std::stringstream s; \
    s << args; \
    RCLCPP_ERROR(swri::get_logger(), "%s", s.str().c_str()); \
  } \
  while (0)

#if NDEBUG
#define ROS_ASSERT(cond) \
  do { \
    if (!(cond)) { \
      ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      /*ROS_ISSUE_BREAK()*/ \
    } \
  } while (false)
#else
#define ROS_ASSERT(cond)
#endif

namespace swri
{
  // The node handle used for all of the loggers
  extern std::shared_ptr<rclcpp::Node> _node_handle;

  /* Call this once in every exectuable with one of the nodes to enable easy logging */
  void setup_logging(std::shared_ptr<rclcpp::Node> ptr);

  rclcpp::Logger get_logger();
}  // namespace swri
#endif  // SWRI_ROSCPP_LOGGING_H_
