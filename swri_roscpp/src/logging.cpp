// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
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

#include <swri_roscpp/logging.h>

#include <rosgraph_msgs/msg/log.hpp>

namespace swri
{
  std::weak_ptr<rclcpp::Node> _node_handle;
  rclcpp::Publisher<rosgraph_msgs::msg::Log>::SharedPtr _log_pub;

  std::mutex _logging_setup_mutex;

  void output_handler(rcutils_log_location_t * location,
        int level, const char * name, const char * format, va_list * args)
  {
    // Output normally as well if the level is high enough
    if (level >= RCUTILS_LOG_SEVERITY_INFO)
    {
      rcutils_logging_console_output_handler(location, level, name, format, args);
    }

    // Format the string to put into the buffer
    char buffer[2048];
    vsnprintf(buffer, sizeof(buffer), format, *args);
          
    // Fill out the ros1 log message
    rosgraph_msgs::msg::Log log;
                  
    auto handle = _node_handle.lock();
    if (handle)
    {
      log.header.stamp = handle->now();
    }

    if (level == RCUTILS_LOG_SEVERITY_DEBUG)
    {
      log.level = rosgraph_msgs::msg::Log::DEBUG;
    }
    else if (level == RCUTILS_LOG_SEVERITY_INFO)
    {
      log.level = rosgraph_msgs::msg::Log::INFO;
    }
    else if (level == RCUTILS_LOG_SEVERITY_WARN)
    {
      log.level = rosgraph_msgs::msg::Log::WARN;
    }
    else if (level == RCUTILS_LOG_SEVERITY_ERROR)
    {
      log.level = rosgraph_msgs::msg::Log::ERROR;
    }
    else
    {
      log.level = rosgraph_msgs::msg::Log::FATAL;
    }
    log.name = name;
    log.msg = buffer;
    log.file = location->file_name;
    log.line = location->line_number;
    log.function = location->function_name;
    _log_pub->publish(log);
  }

  void setup_logging(std::shared_ptr<rclcpp::Node> ptr)
  {
    _logging_setup_mutex.lock();
    if (!_node_handle.lock())
    {
      // Set up a handler and publisher to push these out to /rosout like ROS1 logs
      _log_pub = ptr->create_publisher<rosgraph_msgs::msg::Log>("/rosout", 1);

      _node_handle = ptr;

      rcutils_logging_set_output_handler(output_handler);
    }
    _logging_setup_mutex.unlock();
  }

  rclcpp::Logger get_logger()
  { 
    // TODO: make this not rely on naming for telling if initialized
    static rclcpp::Logger log = rclcpp::get_logger(".LoggerNotSetup");
    if (log.get_name()[0] == '.')// A bit of a hack to tell if we have set up the logger
    {
      // Set up the logger if we have a node handle
      auto handle = _node_handle.lock();
      if (handle)
      {
        log = handle->get_logger();
        ROS_INFO("Logger Setup");
      }
      else
      {
        _node_handle.reset();
      }
    }
    return log;
  }
}
