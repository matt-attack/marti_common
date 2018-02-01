

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
  
  std::string node_name = argc > 1 ? argv[1] : "";

  std::vector<std::string> subscribers;
  std::vector<std::string> publishers;

  auto node = rclcpp::Node::make_shared("node_info_cli");
  auto client = node->create_client<swri_roscpp::srv::Interrogate>(node_name+"/info");
  //using namespace std::chrono_literals;
  /*while (!client->wait_for_service(1s)) 
  {
    if (!rclcpp::ok()) 
    {
      printf(
        "Interrupted while waiting for the service. Exiting.\n");
      return 1;
    }
    printf("Service not available, waiting again...\n");
  }*/

  auto request = std::make_shared<swri_roscpp::srv::Interrogate::Request>();
  /*request->package_name = argv[1];
  request->plugin_name = argv[2];
  for (int i = 4; i < argc; i++)
  {
    RCLCPP_INFO(node->get_logger(), "Sending parameter: %s", argv[i]);
    request->parameters.push_back(argv[i]);
  }*/

  using ServiceResponseFuture =
        rclcpp::Client<swri_roscpp::srv::Interrogate>::SharedFuture;
    auto response_received_callback = [node_name](ServiceResponseFuture future) {
        auto res = future.get();
        printf("got callback");
        //rclcpp::shutdown();
        for (int i = 0; i < res->subscriptions.size(); i++)
        {
          printf("%s Subscribes to: %s (%s)\n", node_name.c_str(), res->subscriptions[i].c_str(),
            res->subscription_types[i].c_str());
        }
      };


  printf("Sending request...\n");
  auto result = client->async_send_request(request, response_received_callback);
  printf("Waiting for response...\n");

  for (int i = 0; i < 100; i++)
  {
    //rclcpp::sleep_for(5000000ns);
    rclcpp::spin_some(node);
  }
  
  /*if (rclcpp::spin_until_future_complete(node, result) !=
      rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    printf("Interrupted while waiting for response. Exiting.\n");
    if (!rclcpp::ok()) 
    {
      return 0;
    }
    return 1;
  }*/
  /*auto res = result.get();
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
  }*/
  //RCLCPP_INFO(
  //  node->get_logger(), "Result of load_node: success = %s",
  //  result.get()->success ? "true" : "false")

  rclcpp::shutdown();

  return 0;//result.get()->success ? 0 : 1;
}
