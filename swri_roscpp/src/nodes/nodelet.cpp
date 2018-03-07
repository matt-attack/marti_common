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

#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.h>

#include <swri_roscpp/node.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/cmdline_parser.h>

#include <swri_roscpp/srv/load_node.hpp>

using namespace std::chrono;

void print_usage()
{
  printf("Usage for nodelet:\n");
  printf("nodelet package_name plugin_name manager_name [--delay delay_ms] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--delay delay_ms: Delay in ms before attempting request. Defaults to 0.\n");
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 3 || rcutils_cli_option_exist(argv, argv + argc, "-h")) 
  {
    print_usage();
    return 0;
  }

  if (rcutils_cli_option_exist(argv, argv + argc, "--delay")) 
  {
    std::chrono::milliseconds delay = std::chrono::milliseconds(
      std::stoul(rcutils_cli_get_option(argv, argv + argc, "--delay")));
    std::this_thread::sleep_for(delay);
  }
  else
  {
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  rclcpp::init(argc, argv);

  if (argc > 3 && std::string(argv[3]) == "standalone")
  {
    rclcpp::Logger logger = rclcpp::get_logger("StandaloneNodelet");

    rclcpp::executors::SingleThreadedExecutor exec;
    std::vector<std::shared_ptr<swri::Node>> nodes;

    // get path to the nodes install location to find the library
    std::string base_path;
    try
    {
      base_path = ament_index_cpp::get_package_prefix(argv[1]);
    }
    catch (ament_index_cpp::PackageNotFoundError err)
    {
      RCLCPP_ERROR(logger, "Could not find requested package '%s'", argv[1])
      return 1;
    }
     
    std::string class_name = argv[2];

    // load node plugin
    std::string library_path = base_path + "/lib/lib"+argv[1]+".so";

    RCLCPP_INFO(logger, "Load library %s", library_path.c_str())
    class_loader::ClassLoader* loader;
    try 
    {
      loader = new class_loader::ClassLoader(library_path);
    } 
    catch (const std::exception& ex) 
    {
      RCLCPP_ERROR(logger, "Failed to load library: %s", ex.what())
      return 1;
    } 
    catch (...) 
    {
      RCLCPP_ERROR(logger, "Failed to load library")
      return 1;
    }
    auto classes = loader->getAvailableClasses<swri::Node>();
    for (auto clazz: classes) 
    {
      RCLCPP_INFO(logger, "Found class %s", clazz.c_str())
      if (clazz == class_name) 
      {
        RCLCPP_INFO(logger, "Instantiating class %s", clazz.c_str())
        auto node = loader->createInstance<swri::Node>(clazz);
        node->Initialize(argc-4, &argv[4], false);
        exec.add_node(node->nh_);
        nodes.push_back(node);
        break;
      }
    }

    exec.spin();

    for (auto node: nodes) 
    {
      exec.remove_node(node->nh_);
    }
    nodes.clear();

    rclcpp::shutdown();

    return 0;
  }
  
  std::string manager_name = argc > 3 ? argv[3] : "standalone";

  std::string noden = argv[2];
  std::replace(noden.begin(), noden.end(), ':', '_');
  std::string node_name = "nodelet_cli"+noden;
  auto node = rclcpp::Node::make_shared(node_name.c_str());
  rmw_qos_profile_t prof = rmw_qos_profile_services_default;
  prof.depth = 1;
  auto client = node->create_client<swri_roscpp::srv::LoadNode>(manager_name+"/load_node", prof);
  //using namespace std::chrono_literals;
  while (!client->wait_for_service(std::chrono::duration<int64_t, std::milli>(1000)))//1s)) 
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Interrupted while waiting for the service. Exiting.")
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Nodelet manager '%s' service not available, waiting again...",
      manager_name.c_str())
    rclcpp::spin_some(node);
  }

  //std::this_thread::sleep_for(std::chrono::milliseconds(100));//give it a bit so the request goes through

  auto request = std::make_shared<swri_roscpp::srv::LoadNode::Request>();
  request->package_name = argv[1];
  request->plugin_name = argv[2];
  for (int i = 4; i < argc; i++)
  {
    //RCLCPP_INFO(node->get_logger(), "Sending parameter: %s", argv[i]);
    request->parameters.push_back(argv[i]);
  }

  RCLCPP_INFO(node->get_logger(), "Sending request...");
  auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Waiting for response...");
  while(true)
  {
    auto ret = rclcpp::spin_until_future_complete(node, result, std::chrono::milliseconds(1000));
    if (ret == rclcpp::executor::FutureReturnCode::SUCCESS) {
      break;
    }
    else if (ret != rclcpp::executor::FutureReturnCode::TIMEOUT)
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.");
      if (!rclcpp::ok()) 
      {
        return 0;
      }
      return 1;
    }
    else
    {
      RCLCPP_INFO(node->get_logger(), "Return code TIMEOUT");
    }

    RCLCPP_INFO(node->get_logger(), "Response not available, waiting again...");
  }
  bool success = result.get()->success;
  if (success == false)
  {
    RCLCPP_ERROR(
      node->get_logger(), "Nodelet failed to load!");
  }
  else
  {
    RCLCPP_INFO(
      node->get_logger(), "Nodelet loaded successfully, exiting launcher");
  }
  //node.reset();
  rclcpp::shutdown();

  printf("Finished nodelet shutdown");

  return success ? 0 : 1;
}
