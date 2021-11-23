#pragma once

#include <deque>
#include <vector>
#include <memory>
#include <utility>
#include <iostream>

#include "asio.hpp"

#include "ts_deque.hpp"

namespace tcp_bridge {
    class Session : public std::enable_shared_from_this<Session> {
    public:
        Session(asio::ip::tcp::socket             socket,
                std::size_t                       buffer_size,
                tcp_bridge::TSDeque<std::string>* read_deque_ptr,
                tcp_bridge::TSDeque<std::string>* write_deque_ptr
        ) : socket_(std::move(socket)),
            read_deque_ptr_(read_deque_ptr),
            write_deque_ptr_(write_deque_ptr) {
            data_.resize(buffer_size);
        }

        void start() { do_read(); }

    protected:
        virtual void on_reading(asio::ip::tcp::socket&) {
            read_deque_ptr_->push_back(std::string(data_.begin(), data_.end()));
            do_write();
        }

    private:
        void do_read() {
            auto self(shared_from_this());
            socket_.async_read_some(asio::buffer(data_), [this, self](std::error_code ec, std::size_t byte) {
                if (!ec) {
                    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Session] "
                        << "Read " << byte << " bytes from: " << socket_.remote_endpoint() << "\n";
                    on_reading(socket_);
                } else {
                    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Session] "
                        << "Error reading from " << socket_.remote_endpoint() << ": " << ec.message() << "\n";
                    socket_.close();
                }
            });
        }

        void do_write() {
            if (write_deque_ptr_->empty()) {
                do_read();
                return;
            }
            auto self(shared_from_this());
            asio::async_write(socket_, asio::buffer(write_deque_ptr_->front()), [this, self](std::error_code ec, std::size_t byte) {
                if (!ec) {
                    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Session] "
                        << "Written " << byte << " bytes from: " << socket_.remote_endpoint() << "\n";
                    write_deque_ptr_->pop_front();
                    do_write();
                } else {
                    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " Session] "
                        << "Error writing to " << socket_.remote_endpoint() << ": " << ec.message() << "\n";
                    socket_.close();
                }
            });
        }

        asio::ip::tcp::socket             socket_;
        std::vector<char>                 data_;

        tcp_bridge::TSDeque<std::string>* read_deque_ptr_;
        tcp_bridge::TSDeque<std::string>* write_deque_ptr_;
    };

    template <typename T>
    class Server {
    public:
        Server(asio::io_context&                 io_context,
               uint16_t                          port,
               std::size_t                       request_size,
               tcp_bridge::TSDeque<std::string>* read_deque_ptr,
               tcp_bridge::TSDeque<std::string>* write_deque_ptr
        ) : acceptor_(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), port)),
            socket_(io_context),
            request_size_(request_size),
            read_deque_ptr_(read_deque_ptr),
            write_deque_ptr_(write_deque_ptr) {
            do_accept();
        }

    private:
        void do_accept() {
            acceptor_.async_accept(socket_, [this](std::error_code ec) {
                if (!ec) {
                    std::cout << "[" << std::chrono::system_clock::now().time_since_epoch().count() << " TCPServer] "
                        << "Accepted connection from: " << socket_.remote_endpoint() << "\n";
                    std::make_shared<T>(std::move(socket_), request_size_, read_deque_ptr_, write_deque_ptr_)->start();
                }
                do_accept();
            });
        }

        asio::ip::tcp::acceptor           acceptor_;
        asio::ip::tcp::socket             socket_;

        std::size_t                       request_size_;
        
        tcp_bridge::TSDeque<std::string>* read_deque_ptr_;
        tcp_bridge::TSDeque<std::string>* write_deque_ptr_;
    };
}
