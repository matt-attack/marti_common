

#include <swri_roscpp/node.h>

namespace swri
{

  void Node::Initialize(int argc, char** argv, bool is_nodelet)
  {
    for (int i = 0; i < argc; i++)
    {
      std::string val = argv[i];
      if (val.find(":=") == -1 || val.length() < 4)
        continue;

      int split = val.find(":=");
      std::string name = val.substr(0, split);
      std::string value = val.substr(split+2);

      // Check for special case names
      if (name == "__name")
      {
        node_name_ = value;
        continue;
      }
      if (name == "__namespace")
      {
        node_namespace_ = value;
        continue;
      }
    }

    nh_ = std::make_shared<rclcpp::Node>(node_name_, node_namespace_, is_nodelet);
    swri::setup_logging(nh_);

    // host a parameter service for each node
    parameter_service_ = std::make_shared<rclcpp::ParameterService>(nh_);
    //auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(nh_);

    parse_arguments(argc, argv);

    onInit();
  }

  void Node::parse_arguments(int argc, char** argv)
  {
    for (int i = 0; i < argc; i++)
    {
      std::string val = argv[i];
      if (val.length() < 4)
      {
        continue;
      }
     
      if (val.find(";=") != -1)
      {
        parse_remap(val);
        continue;
      }

      if (val.find(":=") == -1)
      {
        continue;
      }

      int split = val.find(":=");
      std::string name = val.substr(0, split);
      std::string value = val.substr(split+2);
      ROS_INFO("Set param '%s' with value '%s'", name.c_str(), value.c_str());

      // Check for special case names
      if (name == "__name" || name == "__namespace")
      {
        //node_name_ = name;
        continue;
      }

      // Check which type it is
      if (value == "true")
      {
        //ROS_INFO("Was bool");
        nh_->set_parameter_if_not_set(name, true);
      }
      else if (value == "false")
      {
        //ROS_INFO("Was bool");
        nh_->set_parameter_if_not_set(name, false);
      }
      else if (value[0] == '-' || (value[0] >= '0' && value[0] <= '9'))
      {
        if (value.find('.') == std::string::npos)//integer
        {
          //ROS_INFO("Was int");
          nh_->set_parameter_if_not_set(name, std::atoi(value.c_str()));
        }
        else
        {
          //ROS_INFO("Was float");
          nh_->set_parameter_if_not_set(name, std::atof(value.c_str()));
        }
      }
      else
      {
        //ROS_INFO("Was string");
        nh_->set_parameter_if_not_set(name, value);
      }
    }
  }

  void Node::parse_remap(const std::string& val)
  {
    int split = val.find(";=");
    std::string name = val.substr(0, split);
    std::string value = val.substr(split+2);
    ROS_INFO("Got remap '%s' with value '%s'", name.c_str(), value.c_str());

    remappings_[name] = value;
  }
}
