
//g++ -std=c++11 server.cpp -o server -lboost_system -pthread
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using namespace std;
using namespace boost;
using namespace boost::asio;

class connection
{
public:
  connection(boost::asio::io_service& io)
    : sock_(io),
      KeepAlive_(io)
  {
  }

  tcp::socket& socket()
  {
    return sock_;
  }

  void start()
  { //Iniciates the various processes

    start_KeepAlive();
    start_read_connection();

  }

  void stop()
  {
    stopped_ = true;
    boost::system::error_code ignored_ec;
    sock_.close(ignored_ec);
    //deadline_.cancel();
    //heartbeat_timer_.cancel();
  }

private:
  tcp::socket sock_;
  bool stopped_;
  boost::asio::deadline_timer KeepAlive_;
  boost::asio::streambuf input_buffer_;
  std::string msg_;
  enum { max_length = 1024 };
  char data_[max_length];

  void start_KeepAlive(){
    if (stopped_){
      return;
    }

    else{
    async_write(sock_, buffer("\n",1),
    boost::bind(&connection::handle_KeepAlive, this, _1));

    }
  }
  void handle_KeepAlive(const boost::system::error_code& ec){
    if (stopped_){
      return;
    }
    if (!ec){
      KeepAlive_.expires_from_now(boost::posix_time::seconds(30));
      KeepAlive_.async_wait(boost::bind(&connection::start_KeepAlive, this));
    }
    else{
      std::cout << "Client disconnected" << "\n";
      stop();
    }
  }

  void start_read_connection(){

    boost::asio::async_read_until(sock_, input_buffer_, '\n',
        boost::bind(&connection::handle_read_connection, this, _1));

  }

  void handle_read_connection(const boost::system::error_code& ec){
    if (stopped_)
      return;

    if (!ec)
    {
      // Extract the newline-delimited message from the buffer.
      std::string line;
      std::istream is(&input_buffer_);
      std::getline(is, line);

      // Empty messages are heartbeats and so ignored.
      if (!line.empty())
      {
        std::cout << "Received: " << line << "\n";
      }
    }
    start_read_connection();
  }



};

class server
{
public:
  server(boost::asio::io_service& io_service, short port)
    : io_service_(io_service),
      acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
  {
    start_accept();
  }

private:
  boost::asio::io_service& io_service_;
  tcp::acceptor acceptor_;

  void start_accept()
  {
    connection* new_connection = new connection(io_service_);
    acceptor_.async_accept(new_connection->socket(),
        boost::bind(&server::handle_accept, this, new_connection,
          boost::asio::placeholders::error));
  }

  void handle_accept(connection* new_connection, const boost::system::error_code& ec)
  {
    if (!ec){
      new_connection->start();
    }
    else{
      delete new_connection;
    }
    start_accept();
  }


};

int main(int argc, char* argv[])
{
  int PORT=10000;
  try
  {
    if (argc != 2)
    {
      std::cerr << "Default port used (PORT:10000)\n";
    }
    else{
      PORT=atoi(argv[1]);
    }

    boost::asio::io_service io;
    server s(io, PORT);

    io.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
