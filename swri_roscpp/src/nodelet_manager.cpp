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

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.h>
#include <rclcpp/rclcpp.hpp>

#include <swri_roscpp/node.h>

#include <swri_roscpp/srv/load_node.hpp>

std::mutex thread_mutex;

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // The first optional argument is the name of the nodelet manager
  std::string manager_name = argc > 1 ? argv[1] : "nodelet_manager";

  // Create our node and add it to the executor
  auto node = rclcpp::Node::make_shared(manager_name);
  swri::setup_logging(node);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::vector<std::shared_ptr<swri::Node>> nodes;

  rmw_qos_profile_t prof = rmw_qos_profile_services_default;
  prof.depth = 1;
  auto server = node->create_service<swri_roscpp::srv::LoadNode>(
    manager_name+"/load_node",
    [&exec, &nodes, &node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<swri_roscpp::srv::LoadNode::Request> request,
      std::shared_ptr<swri_roscpp::srv::LoadNode::Response> response)
    {
      // Currently only letting this happen in one thread at a time in an attempt to mitigate thread
      // safety issues in ROS

      thread_mutex.lock();

      // Get the path to the library directory so we can load it
      std::string base_path;
      try
      {
        base_path = ament_index_cpp::get_package_prefix(request->package_name);
      }
      catch (ament_index_cpp::PackageNotFoundError err)
      {
        RCLCPP_ERROR(node->get_logger(), "Could not find requested package '%s'", request->package_name.c_str())
        response->success = false;
        return;
      }
     
      std::string class_name = request->plugin_name;
      RCLCPP_INFO(node->get_logger(), "Loading class: %s", request->plugin_name.c_str());

      // Try and load the given library
      std::string library_path = base_path + "/lib/lib"+request->package_name+".so";

      RCLCPP_INFO(node->get_logger(), "Load library %s", library_path.c_str())
      class_loader::ClassLoader* loader;
      try 
      {
        loader = new class_loader::ClassLoader(library_path);
      } 
      catch (const std::exception & ex) 
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to load library: %s", ex.what())
        response->success = false;
        return;
      } 
      catch (...) 
      {
        RCLCPP_ERROR(node->get_logger(), "Failed to load library")
        response->success = false;
        return;
      }
      // Look for the specified class and if we find it, instantiate and load it
      auto classes = loader->getAvailableClasses<swri::Node>();
      for (auto clazz : classes) 
      {
        if (clazz == class_name) 
        {
          RCLCPP_INFO(node->get_logger(), "Instantiate class %s", clazz.c_str())
          
          auto new_node = loader->createInstance<swri::Node>(clazz);

          // Initialize and pass in the command line arguments that contain params
          // and remappings
          char** strings = new char*[request->parameters.size()];
          for (int i = 0; i < request->parameters.size(); i++)
          {
            strings[i] = new char[request->parameters[i].length()+1];
            strcpy(strings[i], request->parameters[i].c_str());
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(500));// Just for testing purposes
          new_node->Initialize(request->parameters.size(), strings, true);
          for (int i = 0; i < request->parameters.size(); i++)
          {
            delete[] strings[i];
          }
          delete[] strings;

          // Add it to the executor and our other lists
          exec.add_node(new_node->nh_);
          nodes.push_back(new_node);
          delete loader;

          thread_mutex.unlock();
          response->success = true;
          RCLCPP_INFO(node->get_logger(), "Finished loading class: %s", clazz.c_str());
          return;
        }
      }

      // Error if the loader couldnt find the specified class in the given library
      delete loader;
      RCLCPP_ERROR(
        node->get_logger(), "Failed to find class with the requested plugin name '%s' in "
        "the loaded library",
        request->plugin_name.c_str())
      response->success = false;
      thread_mutex.unlock();
      return;
    }, prof);

  exec.spin();

  for (auto node : nodes) 
  {
    exec.remove_node(node->nh_);
  }
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}
