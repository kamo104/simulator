#pragma once

#include "common.h"
#include "sessionState.h"

class Session : public std::enable_shared_from_this<Session> {
  websocket::stream<beast::tcp_stream> _ws;
  net::strand<net::io_context::executor_type> _strand;

  std::shared_ptr<SessionState> _state;
  beast::flat_buffer _recvBuffer;

public:
  explicit Session(tcp::socket &&socket, net::io_context &ioc,
                   std::shared_ptr<SessionState> state)
      : _ws(std::move(socket)), _strand(net::make_strand(ioc)), _state(state) {}

  // Start the asynchronous operation
  void run() {
    _ws.set_option(
        websocket::stream_base::timeout::suggested(beast::role_type::server));

    _ws.set_option(
        websocket::stream_base::decorator([](websocket::response_type &res) {
          res.set(http::field::server, "simulator websocket-server");
        }));

    _ws.async_accept(
        beast::bind_front_handler(&Session::on_accept, shared_from_this()));
  }

  void on_accept(beast::error_code ec) {
    if (ec) {
      _state->isConnected = false;
      return fail(ec, "accept");
    }
    _state->isConnected = true;
    do_read();
    _state->isConnected = false;
    _state->disconnectCallback();
  }

  // void send(const std::string &message) {
  //   net::post(_strand, [self = shared_from_this(), &message]() {
  //     beast::flat_buffer buffer;
  //     auto mutable_buffer = buffer.prepare(message.size());
  //     std::memcpy(mutable_buffer.data(), message.data(), message.size());
  //     buffer.commit(message.size());
  //     self->_ws.async_write(
  //         buffer.data(), beast::bind_front_handler(&Session::on_write,
  //         self));
  //   });
  // }

  void send(const std::string &message) {
    // Copy the message data into a shared buffer
    auto buffer = std::make_shared<beast::flat_buffer>();
    auto mutable_buffer = buffer->prepare(message.size());
    std::memcpy(mutable_buffer.data(), message.data(), message.size());
    buffer->commit(message.size());

    // Post the work to the strand
    net::post(_strand, [self = shared_from_this(), buffer]() {
      self->_ws.async_write(
          buffer->data(),
          [self, buffer](beast::error_code ec, std::size_t bytes_transferred) {
            self->on_write(ec, bytes_transferred);
          });
    });
  }

  void do_read() {
    net::post(_strand, [self = shared_from_this()]() {
      self->_ws.async_read(self->_recvBuffer,
                           beast::bind_front_handler(&Session::on_read, self));
    });
  }

  void on_read(beast::error_code ec, std::size_t bytesTransferred) {
    if (ec == websocket::error::closed) {
      return;
    }

    if (ec)
      fail(ec, "read");
    _ws.text(_ws.got_text());

    std::string msg(beast::buffers_to_string(_recvBuffer.cdata()));
    _recvBuffer.consume(bytesTransferred);
    _state->readCallback(msg);

    do_read();
  }

  void on_write(beast::error_code ec, std::size_t bytesTransferred) {
    if (ec)
      return fail(ec, "write");

    _state->writeCallback(bytesTransferred);
  }
};
