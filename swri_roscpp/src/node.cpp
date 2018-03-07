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

#include <swri_roscpp/node.h>

#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rmw_fastrtps_cpp/get_subscriber.hpp"

#include <swri_roscpp/time.h>
#include <swri_yaml_util/yaml_util.h>

namespace swri
{
  static std::thread timer_updater;
  static std::mutex timers_mutex;

  // Structure containing info about each ROS time based timer we need to update
  struct TimerWatch
  {
    std::weak_ptr<rclcpp::TimerBase> timer;//timer that is set to run at crazy high rate that is only triggered one at a time
    std::weak_ptr<rclcpp::Node> node;
    rclcpp::Duration period;
    rclcpp::Time last_run;
  };
  static std::vector<TimerWatch> timers;

  void Node::interrogate_callback( 
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
          res = res.substr(res.find("tion") + 4, res.length());
          response->subscription_types.push_back(res);
        }
      }

      for (auto client : cbh->get_client_ptrs())
      {
        auto subp = client.lock();
        if (subp)
        {
          response->service_clients.push_back("/" + node_namespace_ + "/"+subp->get_service_name());
          std::string res = typeid(*subp).name();//this->topic_type_map_[pt->get_topic_name()];
          //remove the Subscription part
          res = res.substr(res.find("Client") + 5, res.length());
          response->service_client_types.push_back(res);
        }
      }

      for (auto serv : cbh->get_service_ptrs())
      {
        auto subp = serv.lock();
        if (subp)
        {
          response->service_servers.push_back("/" + node_namespace_ + "/"+subp->get_service_name());
          std::string res = typeid(*subp).name();//this->topic_type_map_[pt->get_topic_name()];
          //remove the Subscription part
          res = res.substr(res.find("Service") + 6, res.length());
          response->service_server_types.push_back(res);
        }
      }
    }

    //for (auto ii: subs_)
    //{
    //  if (auto pt = ii.lock())
    //  {
    //    response->subscriptions.push_back(pt->get_topic_name());
    //    auto res = this->topic_type_map_[pt->get_topic_name()];
    //   response->subscription_types.push_back(res);
    //  }
    //}

    for (auto ii: pubs_)
    {
      if (auto pt = ii.lock())
      {
        response->publications.push_back(pt->get_topic_name());
        auto res = topic_type_map_[pt->get_topic_name()];
        response->publication_types.push_back(res);
      }
    }
  }

  void Node::Initialize(int argc, char** argv, bool is_nodelet)
  {
    // Scan the arguments for the node's name and namespace
    for (int i = 0; i < argc; i++)
    {
      //ROS_ERROR("Got arg: %s", argv[i]);
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
    
    // Create the internal node and use it to setup logging
    nh_ = std::make_shared<rclcpp::Node>(node_name_, node_namespace_, is_nodelet);
    swri::setup_logging(nh_);

    // host a parameter service for each node
    //parameter_service_ = std::make_shared<rclcpp::ParameterService>(nh_);

    info_service_ = nh_->create_service<swri_roscpp::srv::Interrogate>(
    node_name_+"/info", std::bind(&swri::Node::interrogate_callback, this,
     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    //auto parameter_client = std::make_shared<rclcpp::AsyncParametersClient>(nh_);

    parse_arguments(argc, argv);
    //ROS_ERROR("Adding Callback"); 

    if (is_nodelet)
    {
      startup_timer_ = create_wall_timer(4.0, 
        [this]() 
        { 
          ROS_ERROR("Timer Callback"); 
          this->onInit(); 
          this->startup_timer_->cancel(); 
          ROS_ERROR("Finished Timer Callback"); 
        });
    }
    else
    {
      onInit();
    }
  }

  void Node::add_timer(std::weak_ptr<rclcpp::TimerBase> timer, double period)
  {
    timers_mutex.lock();

    // Set up the timer updater thread if we haven't already
    if (timer_updater.joinable() == false)
    {
      timer_updater = std::thread([]()
      {
        while (rclcpp::ok())
        {
          // For each timer check if it is ready to run and execute it if so
          timers_mutex.lock();
          for (auto& t : timers)
          {
            auto node = t.node.lock();
            if (!node)
            {
              continue;
            }
            // Check if any timers are ready, if so triger the node to wake up and the timer to run
            auto timer = t.timer.lock();
            if (!timer)
            {
              continue;
            }

            rclcpp::Time now = node->now();

            // Run if we have exceded our period or time has jumped substantially
            if (now > t.last_run + t.period 
                || abs(now.nanoseconds() - t.last_run.nanoseconds()) > t.period.nanoseconds()*50)
            {
              // Don't run on startup
              if (now.nanoseconds() == 0)
              {
                continue;
              }
              // Reset the timer so it will run when we wake it up
              timer->reset();
              t.last_run = now;

              // Actually wake up the node from any spin loops so it can run the callback
              auto lock = node->get_node_base_interface()->acquire_notify_guard_condition_lock();
              auto condition = node->get_node_base_interface()->get_notify_guard_condition();
              rcl_trigger_guard_condition(condition);
            }
          }
          timers_mutex.unlock();

          //sleep for a wee bit so we dont use all the cpu, this affects granularity, but whatever
          rclcpp::sleep_for(std::chrono::nanoseconds(100000));//sleep for 0.1 ms
        }
      });
    }

    // add the node to the timer list
    timers.push_back({timer,
                      std::weak_ptr<rclcpp::Node>(nh_), 
                      swri::Duration(period), 
                      nh_->now()});
    timers_mutex.unlock();
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
     
      // Handle parsing remappings which are given by += instead of :=
      if (val.find("+=") != -1)
      {
        parse_remap(val);
        continue;
      }

      // Don't bother parsing things that dont have a :=
      if (val.find(":=") == -1)
      {
        continue;
      }

      // Get the parameter name and value by splitting the string at the :=
      int split = val.find(":=");
      std::string name = val.substr(0, split);
      std::string value = val.substr(split+2);
      //ROS_INFO("Set param '%s' with value '%s'", name.c_str(), value.c_str());

      // Check for special names and ignore them as they were already handled
      if (name == "__node" || name == "__ns")
      {
        //node_name_ = name;
        continue;
      }

      // Set the parameter and its type based on the value string
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
        // If it has a . it is a float, if it does not, it is an integer
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

  YAML::Node get_from_name(YAML::Node root, const std::string& name)
  {
    //std::cout << "Name: " << name << "\n";
    //std::cout << "Current:\n" << root;
    
    // Get the position of the / separator so we know if it is a namespace
    int sep_pos = name.find("/");
    if (name[0] >= '0' && name[0] <= '9')
    {
      //its a sequence
      int i = std::atoi(name.c_str());
      if (sep_pos <= 0)
        return root[i];
      return get_from_name(root[i], name.substr(sep_pos+1));
    }

    if (sep_pos >= 1)
    {
      std::string group = name.substr(0, sep_pos);
      //std::cout << "pos is " << sep_pos << " " << group << "\n";

      if (!root[group])
      {
        YAML::Node n(YAML::NodeType::Sequence);
        root[group] = n;
      }
      return get_from_name(root[group], name.substr(sep_pos+1));
    }

    // we are at the top level with no more namespaces so just return the node at this index
    //std::cout << "pos is " << sep_pos << "\n";
    return root[name];
  }

  bool Node::get_parameter(const std::string& na, YAML::Node& node)
  {
    // Get the list of all a parameters and parse it out into a yaml structure based on its names
    auto params = nh_->list_parameters({}, 64);

    // create the root node and make sure it is a Sequence just in case
    // it can automatically be upgraded to a map later
    node = YAML::Node(YAML::NodeType::Sequence);
    node[0] = YAML::Node(YAML::NodeType::Sequence);

    int count = 0;
    for (auto name: params.names)
    {
      // Get the value of the ros parameter
      rclcpp::parameter::ParameterVariant param;
      nh_->get_parameter(name, param); 

      //printf("Got parameter %s\n", name.c_str());

      auto type = param.get_type();
      if (name.substr(0, na.length()) != na)
        continue;

      if (name[na.length()] != '/')
        continue;

      count++;

      // Get/create the YAML:Node associated with that name
      YAML::Node n = get_from_name(node, name.substr(na.length() + 1));
      
      // Set the YAML::Node value based on the ROS parameter value and type
      if (type == rclcpp::parameter::ParameterType::PARAMETER_DOUBLE)
        n = param.as_double();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_BOOL)
        n = param.as_bool();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_INTEGER)
        n = param.as_int();
      else if (type == rclcpp::parameter::ParameterType::PARAMETER_STRING)
        n = param.as_string();
    }
    std::cout << node;
    return count != 0;
  }

  void Node::parse_remap(const std::string& val)
  {
    // Use a split at the += to find the from and to names for the remapping
    int split = val.find("+=");
    std::string name = val.substr(0, split);
    std::string value = val.substr(split+2);
    //ROS_INFO("Got remap '%s' with value '%s'", name.c_str(), value.c_str());

    remappings_[name] = value;
  }
}
