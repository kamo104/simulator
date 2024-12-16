#pragma once

#include "common.h"
#include "sessionState.h"

class Session : public std::enable_shared_from_this<Session> {
  websocket::stream<beast::tcp_stream> _ws;
  http::request<http::string_body> _req;
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

    auto self = shared_from_this();
    http::async_read(
        _ws.next_layer(), _recvBuffer, _req,
        [self](beast::error_code ec, std::size_t bytes_transferred) {
          self->on_http_read(ec, bytes_transferred);
        });

    // _ws.async_accept(
    //     beast::bind_front_handler(&Session::on_accept, shared_from_this()));
  }
  void on_http_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
      std::cerr << "HTTP Read Error: " << ec.message() << "\n";
      return;
    }

    // Access the target (path + query string)
    std::string target = _req.target();
    std::cout << "HTTP Target: " << target << "\n";

    std::string query_string;
    auto pos = target.find('?');
    if (pos != std::string::npos) {
      query_string = target.substr(pos + 1); // Get the query part after '?'
    }

    // Parse the query string
    try {
      auto parsed_query = urls::parse_query(query_string);
      if (parsed_query.has_value()) {
        for (auto key : *parsed_query) {
          std::cout << key.key << " : " << key.value << std::endl;
        }
        auto client_id = parsed_query->find("clientId");
        if (client_id != parsed_query->end()) {
          std::cout << "Client ID: " << client_id->value << "\n";
        } else {
          std::cerr << "clientId parameter not found\n";
        }
      }
    } catch (const std::exception &ex) {
      std::cerr << "Error parsing query: " << ex.what() << "\n";
    }

    _ws.async_accept(_req, beast::bind_front_handler(&Session::on_accept,
                                                     shared_from_this()));
  }
  void on_accept(beast::error_code ec) {
    if (ec) {
      return fail(ec, "accept");
    }
    _state->acceptCallback();
    do_read();
  }

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
      _state->disconnectCallback();
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
