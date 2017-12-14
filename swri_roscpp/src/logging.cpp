

#include <swri_roscpp/logging.h>

namespace swri
{
  std::shared_ptr<rclcpp::Node> _node_handle;

  void setup_logging(std::shared_ptr<rclcpp::Node> ptr)
  {
    _node_handle = ptr;
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
  }
}
