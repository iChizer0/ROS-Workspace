#pragma once

#include <string>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ts_deque.hpp"

namespace tcp_bridge {
    class PubSub : public rclcpp::Node {
    public:
        PubSub(
            tcp_bridge::TSDeque<std::string>* pub_deque_ptr,
            tcp_bridge::TSDeque<std::string>* sub_deque_ptr
        ) : Node("tcp_bridge"), pub_deque_ptr_(pub_deque_ptr), sub_deque_ptr_(sub_deque_ptr) {
            using namespace std::chrono_literals;
            publisher_    = create_publisher<std_msgs::msg::String>   ("tcp_bridge_publisher",  10);
            subscription_ = create_subscription<std_msgs::msg::String>("tcp_bridge_subscriber", 10,
                std::bind(&PubSub::subscription_callback, this, std::placeholders::_1)
            );
            publisher_thread_ = std::thread([this]() { publisher_callback(); });
        }

    protected:
        virtual void subscription_callback(std_msgs::msg::String::SharedPtr msg) {
            std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " ROSPubSub] "
                << "Got subscription message from addr: " << &msg->data << "\n";
            sub_deque_ptr_->push_back(std::string(msg->data));
        }
        
        virtual void publisher_callback() {
            pub_deque_ptr_->wait();
            auto message = std_msgs::msg::String();
            message.data = pub_deque_ptr_->front();
            publisher_->publish(std::move(message));
            std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " ROSPubSub] "
                << "Published message from addr: " << &message.data << "\n";
            pub_deque_ptr_->pop_front();
            publisher_callback();
        }
    
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std::thread                                            publisher_thread_;
        tcp_bridge::TSDeque<std::string>*                      pub_deque_ptr_;
        tcp_bridge::TSDeque<std::string>*                      sub_deque_ptr_;
    };
}
