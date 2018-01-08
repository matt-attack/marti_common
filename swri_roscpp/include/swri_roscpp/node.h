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

namespace swri
{
class Node
{
  std::string node_name_;
  std::string node_namespace_;
 
  std::shared_ptr<rclcpp::ParameterService> parameter_service_;
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

 private:
  void parse_arguments(int argc, char** argv);

  void parse_remap(const std::string& val);

  virtual void onInit() = 0;
};  // class Node

}  // namespace swri
#endif  // SWRI_ROSCPP_NODE_H_
