
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

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/cmdline_parser.h>

#include <swri_roscpp/srv/interrogate.hpp>

void print_usage()
{
  printf("Usage for topic_info:\n");
  printf("topic_info topic_name [--delay delay_ms] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("--delay delay_ms: Delay in ms before attempting request. Defaults to 0.\n");
}

struct TopicInfo
{
  std::string node;
  std::string type;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 2 || rcutils_cli_option_exist(argv, argv + argc, "-h")) 
  {
    print_usage();
    return 0;
  }

  rclcpp::init(argc, argv);
  
  std::string topic_name = argv[1];

  auto node = rclcpp::Node::make_shared("node_info_cli");

  rclcpp::sleep_for(std::chrono::duration<int>(1));
  rclcpp::spin_some(node);

  // Create a service client to interrogate each node for its subscriptions and publications
  std::map<std::string,rclcpp::Client<swri_roscpp::srv::Interrogate>::SharedPtr> clients;
  auto services = node->get_node_graph_interface()->get_service_names_and_types();
  for (auto serv : services)
  {
    std::string name = serv.first;
    if (name.find("/info") != -1)
    {
      // Extract the node name from the service name
      name = name.substr(0,name.find("/info"));
      std::cout << "Got node name: " << name << "\n";
      clients[name] = node->create_client<swri_roscpp::srv::Interrogate>(name+"/info");
    }
  }

  auto request = std::make_shared<swri_roscpp::srv::Interrogate::Request>();

  using ServiceResponseFuture =
        rclcpp::Client<swri_roscpp::srv::Interrogate>::SharedFuture;
    
  printf("Sending requests...\n");

  std::vector<TopicInfo> subscribers;
  std::vector<TopicInfo> publishers;

  // Send out the requests
  int response_count = 0;
  std::vector<ServiceResponseFuture> requests;
  for (auto client : clients)
  {
    std::string node_name = client.first;
    // For each node, send it a request for the topics it subscribes and publishes
    // then go through each and see if it matches the list then add those that do
    auto response_received_callback = [node_name, topic_name, &subscribers, &publishers, &response_count]
      (ServiceResponseFuture future) 
      {
        response_count++;//count how many responses we got
        auto res = future.get();
        // if the topic name matches the one we are investigating, add it to our lists
        for (int i = 0; i < res->subscriptions.size(); i++)
        {
          if (res->subscriptions[i] == topic_name)
          {
            subscribers.push_back({node_name, res->subscription_types[i]});
          }
        }
        for (int i = 0; i < res->publications.size(); i++)
        {
          if (res->publications[i] == topic_name)
          {
            publishers.push_back({node_name, res->publication_types[i]});
          }
        }
      };

    // Send the request to the node and hold onto it so it doesn't go away
    requests.push_back(client.second->async_send_request(request, response_received_callback));
  }

  printf("Waiting for response...\n");

  // Wait a while to get as many responses as we can
  for (int i = 0; i < 5; i++)
  {
    // Exit early if we got all of the responses we were expecting
    if (response_count == clients.size())
    {
      std::cout << "Got all responses.\n";
      break;
    }
    rclcpp::sleep_for(std::chrono::duration<int>(1));
    rclcpp::spin_some(node);
  }

  // Todo print out the nodes that didnt respond 

  std::cout << "Subscribers:\n";
  for (auto sub: subscribers)
  {
    std::cout << " " << sub.first << " (" << sub.second << ")\n";
  }

  std::cout << "\nPublishers:\n";
  for (auto pub: publishers)
  {
    std::cout << " " << pub.first << " (" << pub.second << ")\n";
  }

  rclcpp::shutdown();

  return 0;
}
