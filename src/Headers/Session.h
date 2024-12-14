#include "common.h"

class Session;

struct SessionState {
  enum Client { Unknown, Voice, Pilot, Examiner } program{Unknown};
  bool isConnected{false};
  std::string uid;
  std::shared_ptr<Session> session;

  std::function<void(std::string &)> read_cb = [](auto &msg) {
    std::cout << "Message from client: " << msg << std::endl;
  };
  std::function<void(size_t)> write_cb = [](size_t bytesTransfered) {
    std::cout << "Transfered bytes: " << bytesTransfered << std::endl;
  };
};

struct ServerState {
  std::unordered_map<std::string, std::shared_ptr<SessionState>> sessionMap;
  std::vector<std::string> voiceSessions;
};

class WebsocketServer {
  ServerState serverState;

public:
  WebsocketServer() = default;
  void newSession(std::string uid, std::shared_ptr<SessionState> session) {
    serverState.sessionMap.emplace(uid, session);
  }

  void send(std::string uid, std::string message) {
    auto it = serverState.sessionMap.find(uid);
    if (it == serverState.sessionMap.end()) {
      return;
    }
    it->second->session.send(message);
  }
  void broadcastVoice(std::vector<uint8_t> data) {
    // sessionMap.find()
  }
} websocketServer;

class Session : public std::enable_shared_from_this<Session> {
  websocket::stream<beast::tcp_stream> _ws;
  net::strand<net::io_context::executor_type> _strand;

  std::shared_ptr<SessionState> _state;
  beast::flat_buffer _recvBuffer;

public:
  explicit Session(tcp::socket &&socket, net::io_context &ioc)
      : _ws(std::move(socket)), _strand(net::make_strand(ioc)) {}

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
    if (ec)
      return fail(ec, "accept");

    // initialize();
    // read loop
    do_read();
  }

  void send(const std::string &message) {
    net::post(_strand, [self = shared_from_this(), message]() {
      beast::flat_buffer buffer;
      auto mutable_buffer = buffer.prepare(message.size());
      std::memcpy(mutable_buffer.data(), message.data(), message.size());
      buffer.commit(message.size());
      self->_ws.async_write(
          self->_recvBuffer.data(),
          beast::bind_front_handler(&Session::on_write, self));
    });
  }

  void do_read() {
    net::post(_strand, [self = shared_from_this()]() {
      self->_ws.async_read(self->_recvBuffer,
                           beast::bind_front_handler(&Session::on_read, self));
    });
  }

  void on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec == websocket::error::closed)
      return;

    if (ec)
      fail(ec, "read");
    _ws.text(_ws.got_text());

    std::string buff(beast::buffers_to_string(_recvBuffer.data()));
    _state->read_cb(buff);
    // read_cb();
    // do_read();
  }

  void on_write(beast::error_code ec, std::size_t bytesTransferred) {
    if (ec)
      return fail(ec, "write");

    _state->write_cb(bytesTransferred);
  }
};
