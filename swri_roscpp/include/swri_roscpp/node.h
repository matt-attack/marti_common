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
#ifndef SWRI_ROSCPP_NODE_H_
#define SWRI_ROSCPP_NODE_H_

#include <rclcpp/node.hpp>
#include <swri_roscpp/logging.h>

namespace swri
{
class Node
{
  std::string node_name_;
  std::string node_namespace_;
 public:
  std::shared_ptr<rclcpp::Node> nh_;

 public:
  Node(std::string name) : node_name_(name)
  {

  }

  void Initialize(int argc, char** argv, bool is_nodelet = false)
  {
    for (int i = 0; i < argc; i++)
    {
      std::string val = argv[i];
      if (val.find(":=") == -1 || val.length() < 4)
        continue;

      int split = val.find(":=");
      std::string name = val.substr(0, split);
      std::string value = val.substr(split+2);
      //ROS_INFO("Got param '%s' with value '%s'", name.c_str(), value.c_str());

      // Check for special case names
      if (name == "__NAME__")
      {
        node_name_ = value;
        continue;
      }
      if (name == "__NAMESPACE__")
      {
        node_namespace_ = value;
        continue;
      }
    }

    nh_ = std::make_shared<rclcpp::Node>(node_name_, node_namespace_, is_nodelet);
    swri::setup_logging(nh_);
    parse_arguments(argc, argv);

    onInit();
  }

 private:
  void parse_arguments(int argc, char** argv)
  {
    for (int i = 0; i < argc; i++)
    {
      std::string val = argv[i];
      if (val.find(":=") == -1 || val.length() < 4)
        continue;

      int split = val.find(":=");
      std::string name = val.substr(0, split);
      std::string value = val.substr(split+2);
      ROS_INFO("Got param '%s' with value '%s'", name.c_str(), value.c_str());

      // Check for special case names
      if (name == "__NAME__" || name == "__NAMESPACE__")
      {
        //node_name_ = name;
        continue;
      }

      // Check which type it is
      if (value == "true")
      {
        ROS_INFO("Was bool");
        nh_->set_parameter_if_not_set(name, true);
      }
      else if (value == "false")
      {
        ROS_INFO("Was bool");
        nh_->set_parameter_if_not_set(name, false);
      }
      else if (value[0] == '-' || (value[0] >= '0' && value[0] <= '9'))
      {
        ROS_INFO("Was number");
        nh_->set_parameter_if_not_set(name, std::atof(value.c_str()));
      }
      else
      {
        ROS_INFO("Was string");
        nh_->set_parameter_if_not_set(name, value);
      }
    }
  }

  void apply_params();

  virtual void onInit() = 0;
};  // class Timer

}  // namespace swri
#endif  // SWRI_ROSCPP_NODE_H_
