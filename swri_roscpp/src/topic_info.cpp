

#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.h>

#include <swri_roscpp/node.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcutils/cmdline_parser.h>

#include <swri_roscpp/srv/interrogate.hpp>

using namespace std::chrono;

void print_usage()
{
  printf("Usage for topic_info:\n");
  printf("topic_info topic_name [--delay delay_ms] [-h]\n");
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
  
  std::string topic_name = argc > 1 ? argv[1] : "";

  auto node = rclcpp::Node::make_shared("node_info_cli");
  //auto client = node->create_client<swri_roscpp::srv::Interrogate>(node_name+"/info");
  std::map<std::string,rclcpp::Client<swri_roscpp::srv::Interrogate>::SharedPtr> clients;
  //clients.push_back(client);
  //clients.push_back(node->create_client<swri_roscpp::srv::Interrogate>("/localization/encoder_processor/info"));
  //clients.push_back(node->create_client<swri_roscpp::srv::Interrogate>("/localization/twist_aggregator/info"));

  /*auto names = node->get_node_graph_interface()->get_node_names();
  for (auto name : names)
  {
    std::cout << name << "\n";
  }*/
  rclcpp::sleep_for(std::chrono::duration<int>(1));
  rclcpp::spin_some(node);

  auto services = node->get_node_graph_interface()->get_service_names_and_types();
  for (auto serv : services)
  {
    //std::cout << serv.first << "\n";
    std::string name = serv.first;
    if (name.find("/info") != -1)
    {
      //this is a node name
      name = name.substr(0,name.find("/info"));
      std::cout << "Got node name: " << name << "\n";
      clients[name] = node->create_client<swri_roscpp::srv::Interrogate>(name+"/info");
    }
  }

  auto request = std::make_shared<swri_roscpp::srv::Interrogate::Request>();

  using ServiceResponseFuture =
        rclcpp::Client<swri_roscpp::srv::Interrogate>::SharedFuture;
    
  printf("Sending requests...\n");
  std::vector<std::pair<std::string,std::string>> subscribers;
  std::vector<std::pair<std::string,std::string>> publishers;

  int response_count = 0;
  std::vector<ServiceResponseFuture> requests;
  for (auto client : clients)
  {
    std::string node_name = client.first;
    auto response_received_callback = [node_name, topic_name, &subscribers, &publishers, &response_count](ServiceResponseFuture future) {
        response_count++;
        auto res = future.get();
        for (int i = 0; i < res->subscriptions.size(); i++)
        {
          //printf("%s Subscribes to: %s (%s)\n", node_name.c_str(), res->subscriptions[i].c_str(),
          //  res->subscription_types[i].c_str());
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
    requests.push_back(client.second->async_send_request(request, response_received_callback));
  }
  printf("Waiting for response...\n");

  for (int i = 0; i < 5; i++)
  {
    if (response_count == clients.size())
    {
      std::cout << "Got all responses.\n";
      break;
    }
    rclcpp::sleep_for(std::chrono::duration<int>(1));
    rclcpp::spin_some(node);
  }

  std::cout << "Subscribers:\n";
  for (auto sub: subscribers)
  {
    std::cout << " " << sub.first << " (" << sub.second << ")\n";
  }

  std::cout << "\nPublishers:\n";
  for (auto pub: publishers)
  {
    std::cout << " " << pub.first << " (" << pub.second << "\n";
  }

  rclcpp::shutdown();

  return 0;
}
