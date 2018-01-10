class serial_connection{
private:
  enum {max_length=1024};
  char send_buffer_[max_length];
  serial_port SPort;

public:
  serial_connection(boost::asio::io_service& io);
  void start_serial_connection(std::string port, unsigned int baud_rate);
  void serial_send(std::string line);
  void handle_serial_send(const boost::system::error_code& ec);

};


class socket_connection{
  private:
    tcp::socket sock_;
    boost::asio::deadline_timer KeepAlive_;
    boost::asio::streambuf input_buffer_;
    std::string msg_;
    serial_connection* SPort;
  public:
    socket_connection(boost::asio::io_service& io, serial_connection* SPort_received);
    tcp::socket& socket();
    void start_socket_connection();
    void stop_socket_connection();
    void kill_server();
    void send_socket_connection(std::string msg, bool is_heartbeat);

  private:
    //Function to start the timer to send an heartbeat signal to the client.
    void start_KeepAlive();
    //Function that resets the timer eaxh time a heartbeat signal is sent.
    void handle_KeepAlive(const boost::system::error_code& ec);
    //Function that reads from the socket connection.
    void start_read_socket_connection();
    //Function to handle possible errors from the process of reading the socket.
    void handle_read_socket_connection(const boost::system::error_code& ec);
    //Function to handle possible errors from sending a message to the client
    //through the socket. If a connection was successfull, the heartbeat
    //timer is reset to avoid sending useless heartbeat messages.
    void handle_send_socket_connection(const boost::system::error_code& ec, bool is_heartbeat);
    //Function that performs various tests to ensure a valid command was used.
    bool is_command_valid(std::string &new_line);
};

class server{
  private:
    boost::asio::io_service& io_service_;
    tcp::acceptor acceptor_;
    serial_connection* SPort;
  public:
    server(boost::asio::io_service& io_service, short port, serial_connection* SPort_received);
  private:
    void start_accept();
    void handle_accept(socket_connection* new_socket_connection, const boost::system::error_code& ec);

};
