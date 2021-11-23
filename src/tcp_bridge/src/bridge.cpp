#include <thread>
#include <chrono>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>

#include "tcp_bridge/ts_deque.hpp"
#include "tcp_bridge/ros_pubsub.hpp"
#include "tcp_bridge/tcp_server.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    constexpr uint16_t    tcp_server_port             = 60000;
    constexpr std::size_t tcp_server_recv_buffer_size = 1024;

    tcp_bridge::TSDeque<std::string> message_to_recv;
    tcp_bridge::TSDeque<std::string> message_to_send;

    auto ros_pubsub_ptr = std::make_shared<tcp_bridge::PubSub>(&message_to_recv, &message_to_send);

    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Bridge] "
        << "Spawning ROS publisher and subscription thread...\n";
    auto ros_pubsub_thread = std::thread([&]() {
        rclcpp::spin(ros_pubsub_ptr);
        rclcpp::shutdown();
    });

    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Bridge] "
        << "Spawning TCP server thread...\n";
    auto tcp_server_thread = std::thread([&]() {
        tcp_server_start: try {
            asio::io_context io_context;
            tcp_bridge::Server<tcp_bridge::Session> server(io_context, tcp_server_port, tcp_server_recv_buffer_size, &message_to_recv, &message_to_send);
            io_context.run();
        } catch (std::exception& e) {
            std::cerr << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " TCPServer] "
                << "Restarting server, terminated with exception: " << e.what() << "\n";
            goto tcp_server_start;
        }
    });

    ros_pubsub_thread.join();
    tcp_server_thread.join();

    return 0;
}
