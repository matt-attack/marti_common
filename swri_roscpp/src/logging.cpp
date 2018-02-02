

#include <swri_roscpp/logging.h>

#include <rosgraph_msgs/msg/log.hpp>

namespace swri
{
  std::shared_ptr<rclcpp::Node> _node_handle;
  rclcpp::Publisher<rosgraph_msgs::msg::Log>::SharedPtr _log_pub;

  void setup_logging(std::shared_ptr<rclcpp::Node> ptr)
  {
    if (!_node_handle)
    {
      //set up a handler to publish these logs
      _log_pub = ptr->create_publisher<rosgraph_msgs::msg::Log>("/rosout", 1);

      auto output_handler = [](
        rcutils_log_location_t * location,
        int level, const char * name, const char * format, va_list * args) -> void
        {
          // output normally if we are at the correct level
          if (level >= RCUTILS_LOG_SEVERITY_INFO)
          {
            rcutils_logging_console_output_handler(location, level, name, format, args);
          }

          // Format the string to put into the buffer
          char buffer[1024];
          vsnprintf(buffer, sizeof(buffer), format, *args);
          
          // Fill out the ros1 log message
          rosgraph_msgs::msg::Log log;
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
        };

      _node_handle = ptr;

      rcutils_logging_set_output_handler(output_handler);
    }
  }

  rclcpp::Logger get_logger()
  { 
    // TODO: make this not rely on naming for telling if initialized
    static rclcpp::Logger log = rclcpp::get_logger(".LoggerNotSetup");
    if (log.get_name()[0] == '.' && _node_handle)
    {
      // Set up the logger if we have a node handle
      log = _node_handle->get_logger();
      ROS_INFO("Logger Setup");
    }
    return log;
  }
}
