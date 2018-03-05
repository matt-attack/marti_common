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

#include <swri_roscpp/time.h>
#include <ostream>

namespace swri
{
double toSec(const rclcpp::Time& time)
{
  return static_cast<double>(time.nanoseconds())/1000000000.0;
}

double toSec(const rclcpp::Duration& duration)
{
  return static_cast<double>(duration.nanoseconds())/1000000000.0;
}

rclcpp::Duration Duration(double sec)
{
  int64_t t = (int64_t)sec*1000000000 + fmod(sec, 1)*1000000000.0;
  return rclcpp::Duration(t);
}

rclcpp::Time Time(double sec)
{
  int64_t s = (int64_t)sec*1000000000;
  int64_t n = fmod(sec, 1)*1000000000.0;
  return rclcpp::Time(s, n, RCL_ROS_TIME);
}

const rclcpp::Time TIME_MAX(std::numeric_limits<int32_t>::max(), 999999999, RCL_ROS_TIME);
const rclcpp::Time TIME_MIN(0, 1, RCL_ROS_TIME);

const rclcpp::Duration DURATION_MAX(std::numeric_limits<int32_t>::max(), 999999999);
const rclcpp::Duration DURATION_MIN(0, 1);

std::ostream& operator<<(std::ostream& stream, const rclcpp::Time& t) {
  stream << "Nanos: " << t.nanoseconds();
  return stream;
}

std::ostream& operator<<(std::ostream& stream, const rclcpp::Duration& d) {
  stream << "Nanos: " << d.nanoseconds();
  return stream;
}

}  // namespace swri
