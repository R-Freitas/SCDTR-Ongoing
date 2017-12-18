//g++ -std=c++11 server.cpp -o server -lboost_system -pthread
#include <iostream>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

using boost::asio::deadline_timer;
using namespace boost;
using namespace boost::asio;
using ip::tcp;


class conn :  public enable_shared_from_this<conn> {
public:
  conn(io_service& io)
    :  sock_(io),
      KeepAlive_(io)
    {}

	static shared_ptr<conn> create(io_service& io) {
         return shared_ptr<conn>(new conn(io));
    }
  tcp::socket& socket() {return sock_;}

  void start()
  {
    auto self = shared_from_this();
    async_write(sock_, buffer("Connection established\n"),
    		boost::bind(&conn::handle_connection, shared_from_this()));
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
    boost::asio::streambuf input_buffer_;
   	std::string msg_;
    boost::asio::deadline_timer KeepAlive_;
    boost::system::error_code ec;

	void handle_connection(){
		//Dealing with connection




    start_write();
    start_KeepAlive();
    start_read_connection();


	}
  void start_read_connection(){
    boost::asio::async_read_until(sock_, input_buffer_, '\n',
        boost::bind(&conn::handle_read_connection, this, _1));
        //Starts reading event
  }


  void handle_read_connection(const boost::system::error_code& ec)
  {
    if (stopped_)
      return;

    if (!ec || ec == boost::asio::error::eof)
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

  void start_KeepAlive()
  {
    if (stopped_){
      return;
    }

    else{
    async_write(sock_, buffer("\n",1),
    boost::bind(&conn::handle_KeepAlive, shared_from_this(), _1));

    }
  }

  void handle_KeepAlive(const boost::system::error_code& ec)
  {
    if (stopped_){
      return;
    }
    if (!ec){
      KeepAlive_.expires_from_now(boost::posix_time::seconds(20));
      KeepAlive_.async_wait(boost::bind(&conn::start_KeepAlive, this));
    }
    else{
      std::cout << "Client disconnected" << "\n";
      stop();
    }
  }
  void start_write(){
    if (stopped_){
      return;
    }
    else{
      async_write(sock_, buffer(""),
      boost::bind(&conn::start_write, shared_from_this()));
    }
  }

};

class tcp_server {
private:
    tcp::acceptor acceptor_;
public:
    tcp_server(io_service& io)
     : acceptor_(io, tcp::endpoint(tcp::v4(), 10000))  {
     	start_accept();
     }
private:
   void start_accept() {
       	shared_ptr<conn> new_conn =
        	conn::create(acceptor_.get_io_service());
       	acceptor_.async_accept(new_conn->socket(),
       	[this, new_conn](boost::system::error_code ec) {
       		new_conn->start();
         	start_accept();
		});
   }
};
int main()  try {
    io_service io;
    tcp_server server(io);
    io.run();


} catch(std::exception &e) {std::cout << e.what();}
