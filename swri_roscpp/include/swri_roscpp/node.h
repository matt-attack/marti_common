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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <swri_roscpp/logging.h>

#include <map>

#include <swri_roscpp/srv/interrogate.hpp>

namespace YAML
{
class Node;
}

#define STRINGIZE(x) typeid(x).name()
namespace swri
{
class Node
{
  std::string node_name_;
  std::string node_namespace_;
 
  std::shared_ptr<rclcpp::ParameterService> parameter_service_;
  rclcpp::Service<swri_roscpp::srv::Interrogate>::SharedPtr info_service_;
  std::map<std::string, std::string> remappings_;
 public:
  std::shared_ptr<rclcpp::Node> nh_;

 public:
  Node(std::string name) : node_name_(name)
  {

  }

  std::string ResolveName(const std::string& name) const
  {
    auto iter = remappings_.find(name);
    if (iter != remappings_.end())
    {
      return iter->second; 
    }
    return name;
  }

  void Initialize(int argc, char** argv, bool is_nodelet = false);

  template< class CallbackT>
  rclcpp::TimerBase::SharedPtr create_wall_timer(double rate, CallbackT callback)
  {
    return nh_->create_wall_timer(std::chrono::duration<int64_t, std::micro>((int64_t)(1000000.0/rate)), callback);
  }

  template<class M, class CallbackT>
  rclcpp::SubscriptionBase::SharedPtr create_subscription(const std::string& topic, CallbackT callback,
    const rmw_qos_profile_t& transport_hints)
  {
    auto sub = nh_->create_subscription<M>(topic,
                        callback,
                        transport_hints);
    topic_type_map_[sub->get_topic_name()] = STRINGIZE(M);
    return sub;
  }

  template<class M>
  std::shared_ptr<rclcpp::Publisher<M>> create_publisher(const std::string& topic,
    const rmw_qos_profile_t& transport_hints)
  {
    auto pub = nh_->create_publisher<M>(topic,
                        transport_hints);
    topic_type_map_[pub->get_topic_name()] = STRINGIZE(M);
    pubs_.push_back(pub);
    return pub;
  }

  void get_parameter(const std::string& n, YAML::Node& node);

 private:
  std::map<std::string, std::string> topic_type_map_;
  std::vector<std::weak_ptr<rclcpp::PublisherBase>> pubs_;

  void parse_arguments(int argc, char** argv);

  void parse_remap(const std::string& val);

  virtual void onInit() = 0;
};  // class Node

}  // namespace swri
#endif  // SWRI_ROSCPP_NODE_H_
