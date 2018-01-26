// *****************************************************************************
//
// Copyright (c) 2015, Southwest Research Institute® (SwRI®)
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
#ifndef SWRI_ROSCPP_PUBLISHER_H_
#define SWRI_ROSCPP_PUBLISHER_H_

#include <rclcpp/node.hpp>
#include <swri_roscpp/logging.h>
#include <swri_roscpp/node.h>

namespace swri
{
template<typename M>
std::shared_ptr<rclcpp::Publisher<M>> advertise(
  swri::Node* nh,
  const std::string name,
  uint32_t queue_size,
  bool latched=false)
{
  
  const std::string resolved_name = nh->ResolveName(name);
  if (name == resolved_name)
  {
    std::string ns = nh->nh_->get_namespace();
    std::string real_name = "";
    if (ns != "/")
    {
      real_name +="/";
    }
    real_name += resolved_name;
    ROS_INFO("Publishing [%s] to '%s'.",
           name.c_str(),
           real_name.c_str());
  }
  else
  {
    ROS_INFO("Publishing [%s] to '%s'.",
           name.c_str(),
           resolved_name.c_str());
  }
  
  rmw_qos_profile_t profile = rmw_qos_profile_default;
  if (latched)
  {
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }
  profile.depth = queue_size;
  return nh->create_publisher<M>(resolved_name, profile);
}    
}  // namespace swri
#endif  // SWRI_ROSCPP_PUBLISHER_H_
