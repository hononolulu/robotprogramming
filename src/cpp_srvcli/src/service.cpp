// #include "rclcpp/rclcpp.hpp"
// #include "example_interfaces/srv/add_two_ints.hpp"

// #include <memory>

// void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
//           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>      response)
// {
//   response->sum = request->a + request->b;
//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
//                 request->a, request->b);

//   std::this_thread::sleep_for(std::chrono::seconds(10));

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);

//   std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

//   rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
//     node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

//   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

//   rclcpp::spin(node);
//   rclcpp::shutdown();
// }

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>
#include <thread>
#include <chrono>

void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
          std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("minimal_service"), "Incoming request\na: %ld b: %ld",
                request->a, request->b);

  // 10초 지연
  std::this_thread::sleep_for(std::chrono::seconds(10));

  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending back response: [%ld]", response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service =
    node->create_service<example_interfaces::srv::AddTwoInts>("add_two_ints", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
