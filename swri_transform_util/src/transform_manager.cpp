// *****************************************************************************
//
// Copyright (c) 2014, Southwest Research Institute® (SwRI®)
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
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <swri_transform_util/transform_manager.h>

#include <vector>

#include <swri_transform_util/frames.h>
#include <swri_transform_util/utm_transformer.h>
#include <swri_transform_util/wgs84_transformer.h>

namespace swri_transform_util
{
  TransformManager::TransformManager()
  {
    std::vector<std::shared_ptr<Transformer> > transformers;
    transformers.push_back(std::make_shared<Wgs84Transformer>());
    transformers.push_back(std::make_shared<UtmTransformer>());

    for (size_t i = 0; i < transformers.size(); i++)
    {
      std::shared_ptr<Transformer> transformer = transformers[i];
      std::map<std::string, std::vector<std::string> > supports = transformer->Supports();

      std::map<std::string, std::vector<std::string> >::iterator iter;
      for (iter = supports.begin(); iter != supports.end(); ++iter)
      {
        for (uint32_t j = 0; j < iter->second.size(); j++)
        {
          if (transformers_[iter->first].count(iter->second[j]) > 0)
          {
            printf("WARN: [transform_manager]: Transformer conflict for %s to %s",
                iter->first.c_str(), iter->second[j].c_str());
          }

          transformers_[iter->first][iter->second[j]] = transformer;
        }
      }
    }
  }

  TransformManager::~TransformManager()
  {
  }

  void TransformManager::Initialize(
      std::shared_ptr<rclcpp::Node> handle,
      std::shared_ptr<tf2_ros::Buffer> tf)
  {
    handle_ = handle;
    if (!tf)
    {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, handle, true);
    }
    else
    {
      tf_buffer_ = tf;
    }
    local_xy_util_ = std::make_shared<LocalXyWgs84Util>(handle);

    std::map<std::string, std::map<std::string, std::shared_ptr<Transformer> > >::iterator iter1;
    for (iter1 = transformers_.begin(); iter1 != transformers_.end(); ++iter1)
    {
      std::map<std::string, std::shared_ptr<Transformer> >::iterator iter2;
      for (iter2 = iter1->second.begin(); iter2 != iter1->second.end(); ++iter2)
      {
        iter2->second->Initialize(handle, tf);
      }
    }
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const rclcpp::Time& time,
      Transform& transform) const
  {
    std::string src_frame = source_frame;
    std::string tgt_frame = target_frame;
    if (tgt_frame == src_frame)
    {
      transform = Transform(handle_->now());
      return true;
    }

    if (!tf_buffer_)
    {
      printf("WARN: [transform_manager]: TF listener not initialized.");
      return false;
    }

    // Check if the source frame is in the TF tree.
    std::string source = src_frame;
    if (tf_buffer_->_frameExists(source))
    {
      source = _tf_frame;
    }
    else if (!source.empty() && source[0] == '/' && tf_buffer_->_frameExists(source.substr(1)))
    {
      src_frame = source.substr(1);
      source = _tf_frame;
    }
    else if (!source.empty() && source[0] != '/' && tf_buffer_->_frameExists("/" + source))
    {
      src_frame = "/" + source;
      source = _tf_frame;
    }
    else if (!source.empty() && source[0] != '/')
    {
      source = "/" + source;
    }

    // Check if the target frame is in the TF tree.
    std::string target = tgt_frame;
    if (tf_buffer_->_frameExists(target))
    {
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] == '/' && tf_buffer_->_frameExists(target.substr(1)))
    {
      tgt_frame = target.substr(1);
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] != '/' && tf_buffer_->_frameExists("/" + target))
    {
      tgt_frame = "/" + target;
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] != '/')
    {
      target = "/" + target;
    }

    // Check if either of the frames is local_xy
    if (source == _local_xy_frame)
    {
      source = _tf_frame;

      if (!local_xy_util_->Initialized())
      {
        printf("WARN: [transform_manager]: Local XY frame has not been initialized.");
        return false;
      }

      src_frame = local_xy_util_->Frame();
    }

    if (target == _local_xy_frame)
    {
      target = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        printf("WARN: [transform_manager]: Local XY frame has not been initialized.");
        return false;
      }

      tgt_frame = local_xy_util_->Frame();
    }

    if (source == target)
    {
      // Both frames are in the TF tree.

      geometry_msgs::msg::TransformStamped tf_transform;
      if (GetTransform(tgt_frame, src_frame, time, tf_transform))
      {
        transform = (Transform)tf_transform;
        return true;
      }

      printf("ERROR: [transform_manager]: Failed to get tf transform ('%s' to '%s').  Both "
        "frames exist in tf.",
        source_frame.c_str(), target_frame.c_str());
      return false;
    }

    SourceTargetMap::const_iterator source_iter = transformers_.find(source);
    if (source_iter == transformers_.end())
    {
      printf("WARN: [transform_manager]: No transformer from '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), source.c_str());

      return false;
    }

    TransformerMap::const_iterator target_iter = source_iter->second.find(target);
    if (target_iter == source_iter->second.end())
    {
      printf("WARN: [transform_manager]: No transformer from '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), target.c_str());

      return false;
    }

    std::shared_ptr<Transformer> transformer = target_iter->second;

    if (!transformer)
    {
      printf("ERROR: [transform_manager]: Encountered null transformer for '%s' to '%s'.",
          source.c_str(), target.c_str());

      return false;
    }

    return transformer->GetTransform(tgt_frame, src_frame, time, transform);
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      Transform& transform) const
  {
    return GetTransform(target_frame, source_frame, rclcpp::Time(0), transform);
  }

  bool TransformManager::SupportsTransform(
      const std::string& target_frame,
      const std::string& source_frame) const
  {
    if (target_frame == source_frame)
    {
      return true;
    }

    if (!tf_buffer_)
    {
      return false;
    }

    // Check if the source frame is in the TF tree.
    std::string source = source_frame;
    if (tf_buffer_->_frameExists(source))
    {
      source = _tf_frame;
    }
    else if (!source.empty() && source[0] == '/' && tf_buffer_->_frameExists(source.substr(1)))
    {
      source = _tf_frame;
    }
    else if (!source.empty() && source[0] != '/' && tf_buffer_->_frameExists("/" + source))
    {
      source = _tf_frame;
    }
    else if(!source.empty() && source[0] != '/')
    {
      source = "/" + source;
    }

    // Check if the target frame is in the TF tree.
    std::string target = target_frame;
    if (tf_buffer_->_frameExists(target))
    {
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] == '/' && tf_buffer_->_frameExists(target.substr(1)))
    {
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] != '/' && tf_buffer_->_frameExists("/" + target))
    {
      target = _tf_frame;
    }
    else if (!target.empty() && target[0] != '/')
    {
      target = "/" + target;
    }

    // Check if either of the frames is local_xy
    if (source == _local_xy_frame)
    {
      source = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        printf("WARN: [transform_manager]: Local XY frame has not been initialized.");
        return false;
      }
    }

    if (target == _local_xy_frame)
    {
      target = _tf_frame;
      if (!local_xy_util_->Initialized())
      {
        printf("WARN: [transform_manager]: Local XY frame has not been initialized.");
        return false;
      }
    }

    if (source == target)
    {
      return true;
    }

    SourceTargetMap::const_iterator source_iter = transformers_.find(source);
    if (source_iter == transformers_.end())
    {
      printf("WARN: [transform_manager]: No transformer for transforming '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), source.c_str());

      return false;
    }

    TransformerMap::const_iterator target_iter = source_iter->second.find(target);
    if (target_iter == source_iter->second.end())
    {
      printf("WARN: [transform_manager]: No transformer for transforming '%s' to '%s'."
        " If '%s' is a /tf frame, it may not have been broadcast recently.",
        source.c_str(), target.c_str(), target.c_str());

      return false;
    }

    return true;
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      const rclcpp::Time& time,
      geometry_msgs::msg::TransformStamped& transform) const
  {
    if (!tf_buffer_)
      return false;

    bool has_transform = false;
    try
    {
      //tf_listener_->waitForTransform(
       //   target_frame,
        //  source_frame,
         // time,
          //ros::Duration(0.1));

      transform = tf_buffer_->lookupTransform(
          target_frame,
          source_frame,
          tf2::timeFromSec(time.nanoseconds()/1000000000.0));
          //transform);

      has_transform = true;
    }
    catch (const tf2::LookupException& e)
    {
      printf("ERROR: [transform_manager]: %s", e.what());
    }
    catch (const tf2::ConnectivityException& e)
    {
      printf("ERROR: [transform_manager]: %s", e.what());
    }
    catch (const tf2::ExtrapolationException& e)
    {
      printf("ERROR: [transform_manager]: %s", e.what());
    }
    catch (...)
    {
      printf("ERROR: [transform_manager]: Exception looking up transform");
    }

    return has_transform;
  }

  bool TransformManager::GetTransform(
      const std::string& target_frame,
      const std::string& source_frame,
      geometry_msgs::msg::TransformStamped& transform) const
  {
    return GetTransform(target_frame, source_frame, rclcpp::Time(0), transform);
  }
}
