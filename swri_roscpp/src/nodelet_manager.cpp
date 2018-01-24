
/*#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.h"
#include "rclcpp/rclcpp.hpp"

#include <swri_roscpp/node.h>

#define DLOPEN_COMPOSITION_LOGGER_NAME "dlopen_composition"

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (argc < 2) {
    fprintf(stderr, "Requires at least one argument to be passed with the library to load\n");
    return 1;
  }
  rclcpp::Logger logger = rclcpp::get_logger(DLOPEN_COMPOSITION_LOGGER_NAME);
  rclcpp::init(argc, argv);

  auto nh = std::make_shared<rclcpp::Node>("nodelet_manager");
  swri::setup_logging(nh);
 
  rclcpp::executors::SingleThreadedExecutor exec;
  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<std::shared_ptr<swri::Node>> nodes;

  std::vector<std::string> libraries;
  for (int i = 1; i < argc; ++i) {
    libraries.push_back(argv[i]);
  }
  for (auto library : libraries) {
    RCLCPP_INFO(logger, "Load library %s", library.c_str())
    auto loader = new class_loader::ClassLoader(library);
    auto classes = loader->getAvailableClasses<swri::Node>();
    for (auto clazz : classes) {
      RCLCPP_INFO(logger, "Instantiate class %s", clazz.c_str())
      auto node = loader->createInstance<swri::Node>(clazz);
      node->Initialize(0, 0, true);
      exec.add_node(node->nh_);
      nodes.push_back(node);
    }
    loaders.push_back(loader);
  }
  RCLCPP_INFO(logger, "Spinning...");
  exec.spin();

  for (auto node : nodes) {
    exec.remove_node(node->nh_);
  }
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}*/

#include <string>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <class_loader/class_loader.h>
#include <rclcpp/rclcpp.hpp>

#include <swri_roscpp/node.h>

#include <swri_roscpp/srv/load_node.hpp>

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  std::string manager_name = argc > 1 ? argv[1] : "nodelet_manager";
  auto node = rclcpp::Node::make_shared(manager_name);
  swri::setup_logging(node);
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  std::vector<class_loader::ClassLoader *> loaders;
  std::vector<std::shared_ptr<swri::Node>> nodes;

  auto server = node->create_service<swri_roscpp::srv::LoadNode>(
    manager_name+"/load_node",
    [&exec, &loaders, &nodes, &node](
      const std::shared_ptr<rmw_request_id_t>,
      const std::shared_ptr<swri_roscpp::srv::LoadNode::Request> request,
      std::shared_ptr<swri_roscpp::srv::LoadNode::Response> response)
    {
      // get node plugin resource from package
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

      // load node plugin
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
      auto classes = loader->getAvailableClasses<swri::Node>();
      for (auto clazz : classes) 
      {
        RCLCPP_INFO(node->get_logger(), "Found class %s", clazz.c_str())
        if (clazz == class_name) 
        {
          RCLCPP_INFO(node->get_logger(), "Instantiate class %s", clazz.c_str())
          auto node = loader->createInstance<swri::Node>(clazz);
          char** strings = new char*[request->parameters.size()];
          for (int i = 0; i < request->parameters.size(); i++)
          {
            strings[i] = new char[request->parameters[i].length()+1];
            strcpy(strings[i], request->parameters[i].c_str());
          }
          node->Initialize(request->parameters.size(), strings, true);
          for (int i = 0; i < request->parameters.size(); i++)
          {
            delete[] strings[i];
          }
          delete[] strings;
          exec.add_node(node->nh_);
          nodes.push_back(node);
          loaders.push_back(loader);
          response->success = true;
          return;
        }
      }

      // no matching class found in loader
      delete loader;
      RCLCPP_ERROR(
        node->get_logger(), "Failed to find class with the requested plugin name '%s' in "
        "the loaded library",
        request->plugin_name.c_str())
      response->success = false;
      return;
    });

  exec.spin();

  for (auto node : nodes) 
  {
    exec.remove_node(node->nh_);
  }
  nodes.clear();

  rclcpp::shutdown();

  return 0;
}
