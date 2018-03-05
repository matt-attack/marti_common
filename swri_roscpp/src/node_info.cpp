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

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/cmdline_parser.h>

#include <swri_roscpp/srv/interrogate.hpp>

void print_usage()
{
  printf("Usage for node_info:\n");
  printf("node_info node_name [--delay delay_ms] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--delay delay_ms: Delay in ms before attempting request. Defaults to 0.\n");
}

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) 
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

  rclcpp::init(argc, argv);
  
  std::string node_name = argv[1];

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("node_info_cli");
  auto client = node->create_client<swri_roscpp::srv::Interrogate>(node_name+"/info");

  // Wait until the node's info service has started
  while (!client->wait_for_service(std::chrono::duration<int64_t, std::milli>(1000)) )
  {
    if (!rclcpp::ok()) 
    {
      printf(
        "Interrupted while waiting for the service. Exiting.\n");
      return 1;
    }
    printf("Service not available, waiting again...\n");
  }

  auto request = std::make_shared<swri_roscpp::srv::Interrogate::Request>();

  printf("Sending request...\n");
  auto result = client->async_send_request(request);
  printf("Waiting for response...\n");
  if (rclcpp::spin_until_future_complete(node, result) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("Interrupted while waiting for response. Exiting.\n");
    if (!rclcpp::ok()) 
    {
      return 0;
    }
    return 1;
  }

  auto res = result.get();
  for (int i = 0; i < res->subscriptions.size(); i++)
  {
    printf("Subscribes to: %s (%s)\n", res->subscriptions[i].c_str(),
                res->subscription_types[i].c_str());
  }

  for (int i = 0; i < res->publications.size(); i++)
  {
    printf("Publishes: %s (%s)\n", res->publications[i].c_str(),
                res->publication_types[i].c_str());
  }

  for (int i = 0; i < res->service_servers.size(); i++)
  {
    printf("Service host: %s (%s)\n", res->service_servers[i].c_str(),
                res->service_server_types[i].c_str());
  }

  for (int i = 0; i < res->service_servers.size(); i++)
  {
    printf("Service client: %s (%s)\n", res->service_clients[i].c_str(),
                res->service_client_types[i].c_str());
  }

  rclcpp::shutdown();

  return 0;
}
