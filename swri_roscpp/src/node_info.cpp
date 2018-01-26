

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
  
  std::string node_name = argc > 1 ? argv[1] : "";

  auto node = rclcpp::Node::make_shared("nodelet_cli");
  auto client = node->create_client<swri_roscpp::srv::Interrogate>(node_name+"/info");
  using namespace std::chrono_literals;
  while (!client->wait_for_service(1s)) 
  {
    if (!rclcpp::ok()) 
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Interrupted while waiting for the service. Exiting.")
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...")
  }

  auto request = std::make_shared<swri_roscpp::srv::Interrogate::Request>();
  /*request->package_name = argv[1];
  request->plugin_name = argv[2];
  for (int i = 4; i < argc; i++)
  {
    RCLCPP_INFO(node->get_logger(), "Sending parameter: %s", argv[i]);
    request->parameters.push_back(argv[i]);
  }*/

  RCLCPP_INFO(node->get_logger(), "Sending request...")
  auto result = client->async_send_request(request);
  RCLCPP_INFO(node->get_logger(), "Waiting for response...")
  if (rclcpp::spin_until_future_complete(node, result) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for response. Exiting.")
    if (!rclcpp::ok()) 
    {
      return 0;
    }
    return 1;
  }
  auto res = result.get();
  for (int i = 0; i < res->subscriptions.size(); i++)
  {
    RCLCPP_INFO(node->get_logger(), "Subscribes to: %s (%s)", res->subscriptions[i].c_str(),
                res->subscription_types[i].c_str());
  }

  for (int i = 0; i < res->publications.size(); i++)
  {
    RCLCPP_INFO(node->get_logger(), "Publishes: %s (%s)", res->publications[i].c_str(),
                res->publication_types[i].c_str());
  }
  //RCLCPP_INFO(
  //  node->get_logger(), "Result of load_node: success = %s",
  //  result.get()->success ? "true" : "false")

  rclcpp::shutdown();

  return 0;//result.get()->success ? 0 : 1;
}
