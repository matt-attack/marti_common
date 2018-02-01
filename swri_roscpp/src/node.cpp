

#include <swri_roscpp/node.h>

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_subscriber.hpp"

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
      if (name == "__node")
      {
        node_name_ = value;
        continue;
      }
      if (name == "__ns")
      {
        node_namespace_ = value;
        continue;
      }
    }

    nh_ = std::make_shared<rclcpp::Node>(node_name_, node_namespace_, is_nodelet);
    swri::setup_logging(nh_);

    // host a parameter service for each node
    //parameter_service_ = std::make_shared<rclcpp::ParameterService>(nh_);

    info_service_ = nh_->create_service<swri_roscpp::srv::Interrogate>(
    node_name_+"/info",
    [this](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<swri_roscpp::srv::Interrogate::Request> request,
      std::shared_ptr<swri_roscpp::srv::Interrogate::Response> response)
    {
      for (auto cb: nh_->get_callback_groups())
      {
        auto cbh = cb.lock();
        if (!cbh)
        {
          continue;
        }

        for (auto sub : cbh->get_subscription_ptrs())
        {
          auto subp = sub.lock();
          if (subp)
          {
            //subp
            rcl_subscription_t * rcl_sub = subp->get_subscription_handle();
            rmw_subscription_t * rmw_sub = rcl_subscription_get_rmw_handle(rcl_sub);
            eprosima::fastrtps::Subscriber * p = rmw_fastrtps_cpp::get_subscriber(rmw_sub);
            //auto attribs = p->getAttributes();
            

            response->subscriptions.push_back(subp->get_topic_name());
            std::string res = typeid(*subp).name();//this->topic_type_map_[pt->get_topic_name()];
            //remove the Subscription part
            res = res.substr(res.find("tion")+4, res.length());
            response->subscription_types.push_back(res);
          }
        }

        for (auto client : cbh->get_client_ptrs())
        {
          auto subp = client.lock();
          if (subp)
          {
            response->service_clients.push_back("/"+node_namespace_+"/"+subp->get_service_name());
            std::string res = typeid(*subp).name();//this->topic_type_map_[pt->get_topic_name()];
            //remove the Subscription part
            res = res.substr(res.find("Client")+5, res.length());
            response->service_client_types.push_back(res);
          }
        }

        for (auto serv : cbh->get_service_ptrs())
        {
          auto subp = serv.lock();
          if (subp)
          {
            response->service_servers.push_back("/"+node_namespace_+"/"+subp->get_service_name());
            std::string res = typeid(*subp).name();//this->topic_type_map_[pt->get_topic_name()];
            //remove the Subscription part
            res = res.substr(res.find("Service")+6, res.length());
            response->service_server_types.push_back(res);
          }
        }
      }

      /*for (auto ii: subs_)
      {
        if (auto pt = ii.lock())
        {
          response->subscriptions.push_back(pt->get_topic_name());
          auto res = this->topic_type_map_[pt->get_topic_name()];
          response->subscription_types.push_back(res);
        }
      }*/

      for (auto ii: pubs_)
      {
        if (auto pt = ii.lock())
        {
          response->publications.push_back(pt->get_topic_name());
          auto res = this->topic_type_map_[pt->get_topic_name()];
          response->publication_types.push_back(res);
        }
      }
    });
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
      if (name == "__node" || name == "__ns")
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

  void Node::get_parameter(const std::string& n, YAML::Node& ne)
  {
    auto params = nh_->list_parameters({}, 64);
    auto& names = params.names;
    //auto node = YAML::Load("[1, 2, 3]");
    YAML::Node no;
    no["pie"] = "test";
    for (auto name: names)
    {
      rclcpp::parameter::ParameterVariant param;
      nh_->get_parameter(name, param); 

      printf("Got parameter %s\n", name.c_str());

      auto type = param.get_type();
      
      /*if (type == rclcpp::parameter::ParameterType::PARAMETER_DOUBLE)
        node[name] = param.as_double();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_BOOL)
        node[name] = param.as_bool();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_INTEGER)
        node[name] = param.as_int();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_STRING)
        node[name] = param.as_string();*/

    }
    //std::cout << node;
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
