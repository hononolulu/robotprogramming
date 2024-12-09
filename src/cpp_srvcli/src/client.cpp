#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client");
  auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);

  int counter = 1;

  // 1초마다 실행되는 타이머
  auto timer = node->create_wall_timer(1s, [node, &counter]() {
    RCLCPP_INFO(rclcpp::get_logger("minimal_client_async"), "%d", counter);
    counter++;
  });

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  while (rclcpp::ok()) {
    auto result = client->async_send_request(request);

    // 응답이 올 때까지 기다리지만, 타이머는 계속 동작
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("minimal_client_async"), "Result: %ld", result.get()->sum);
      counter = 0;  // 응답을 받으면 카운터 초기화
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
    }

    // 새로운 요청을 보내기 전에 현재 카운터 값을 유지하고자 함
    rclcpp::sleep_for(1s);
  }

  rclcpp::shutdown();
  return 0;
}
