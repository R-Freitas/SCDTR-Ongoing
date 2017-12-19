//g++ -std=c++11 server_2.cpp -o server -lboost_system -pthread -lpigpio -lrt
#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <thread>
#include <boost/asio.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/lexical_cast.hpp>
#include <pigpio.h>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using namespace std;
using namespace boost;
using namespace boost::asio;

bool stopped_;


//I2C Variables
#define SCL 19
#define SDA 18
#define ADDRESS 0x09
bsc_xfer_t xfer;



//--------------------Miscelaneous functions------------------------------------
//Functions for i2c services

void i2c_handle_read (int event, uint32_t tick){
  int status;
  printf("HEY\n");
  status = bscXfer(&xfer);
  if(xfer.rxCnt !=0)
  {
    std::cout<< xfer.rxBuf<< std::endl;
    std::cout<< "Recebido " << xfer.rxCnt << " bytes" << std::endl;
  }
}

void i2c_read(){
for(;;)
  {
    bscXfer(&xfer);
    xfer.txCnt=0;
    sleep(600);
    if (stopped_){
      return;
    }
  }
}

void initialize_i2c(){
  xfer.control = (0x09<<16) | 0x305;

  if (gpioInitialise() < 0) { printf("Erro 1\n"); return ;}

  gpioSetPullUpDown(18, PI_PUD_UP);
  gpioSetPullUpDown(19, PI_PUD_UP);

  eventSetFunc(PI_EVENT_BSC, i2c_handle_read);

}


class connection
{
public:
  connection(boost::asio::io_service& io)
    : sock_(io),
      KeepAlive_(io),
      SPort(io)
  {
  }

  tcp::socket& socket()
  {
    return sock_;
  }

  void start()
  { //Iniciates the various processes

    start_KeepAlive();
    start_serial();
    start_read_connection();
    initialize_i2c();
    thread t (i2c_read);
    t.detach();
  }

  void stop(){
    stopped_ = true;
    boost::system::error_code ignored_ec;
    sock_.close(ignored_ec);
    KeepAlive_.cancel();
    
    //heartbeat_timer_.cancel();
  }
private:
  tcp::socket sock_;
  boost::asio::deadline_timer KeepAlive_;
  boost::asio::streambuf input_buffer_;
  enum {max_length=1024};
  char send_buffer_[max_length];
  serial_port SPort;
  std::string msg_;



  void start_serial(){
    //Function to initialize serial comunication
    boost::system::error_code error;
    SPort.open("/dev/ttyACM0", error); //connect to port containing arduino
    if( error ) {cout << "Error when connecting to Arduino \n"; return ;}
    SPort.set_option(serial_port_base::baud_rate(9600),error);
    if( error ) {cout << "Error when initializing connection to Arduino \n"; return ;}
  }

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
      KeepAlive_.expires_from_now(boost::posix_time::seconds(28));
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
        //boost::erase_all(line, " ");
        if (is_command_valid(line)){
          handle_serial_send(line);
        }
      }
    }
    start_read_connection();
  }

  void handle_serial_send(std::string line){
    std::string terminated_line;

    std::cout << "Sending: " << line << "\n";
    terminated_line = line + std::string("\n");
    std::size_t n = terminated_line.size();
    terminated_line.copy(send_buffer_, n);
    boost::asio::async_write(SPort, boost::asio::buffer(send_buffer_,n),
          boost::bind(&connection::empty_handle_serial, this, _1));


  }

  void empty_handle_serial(const boost::system::error_code& ec){
    if (ec)
    {
      std::cout << "Error on serial communication to Arduino\n";
    }
  }

//-----------------------------ERROR HANDLING--------------------------------

bool is_command_valid(std::string &new_line){
  std::vector<std::string> cmds;
  std::string msg_;
  enum {max_length=1024};
  char error_buffer_[max_length];

  if(new_line.compare("r")!=0){
    split(cmds, new_line, is_any_of(" ")); // here it is
    if (cmds.size()==3){
        if (cmds[0].compare("s")== 0){
          try{
            int ocup = lexical_cast<int>(cmds[2]);
            lexical_cast<int>(cmds[1]);
            if (ocup!=0 && ocup!=1){
              msg_="Invalid command:Occupancy command is used as 's <desk_i> <val>'\n";
              std::size_t n = msg_.size();
              msg_.copy(error_buffer_, n);
              //write(sock_, boost::asio::buffer(error_buffer_,n));
              boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                    boost::bind(&connection::empty_handle_error, this, _1));
              return false;
            }
          }
          catch(boost::bad_lexical_cast&){
            msg_="Invalid command:Occupancy command is used as 's <desk_i> <val>'\n";
            std::size_t n = msg_.size();
            msg_.copy(error_buffer_, n);
            //write(sock_, boost::asio::buffer(error_buffer_,n));
            boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                  boost::bind(&connection::empty_handle_error, this, _1));
            return false;
          }
          new_line=cmds[0]+' '+cmds[2]+' '+cmds[1];
          return true;
        }

        else if (cmds[0].compare("c") == 0 || cmds[0].compare("d") == 0 ||
                 cmds[0].compare("b") == 0){
          if (cmds[1].compare("l") == 0 || cmds[1].compare("d") == 0){
            try{
              lexical_cast<int>(cmds[2]);
            }
            catch(boost::bad_lexical_cast&){
              msg_="Invalid command:See list provided by the professor\n";
              std::size_t n = msg_.size();
              msg_.copy(error_buffer_, n);
              //write(sock_, boost::asio::buffer(error_buffer_,n));
              boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                    boost::bind(&connection::empty_handle_error, this, _1));
              return false;
            }
            return true;
          }
          else{
            msg_="Invalid command:See list provided by the professor\n";
            std::size_t n = msg_.size();
            msg_.copy(error_buffer_, n);
            //write(sock_, boost::asio::buffer(error_buffer_,n));
            boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                  boost::bind(&connection::empty_handle_error, this, _1));
            return false;
          }
        }

        else if (cmds[0].compare("g") == 0){
          if (cmds[1].compare("p") == 0 || cmds[1].compare("e") == 0 ||
              cmds[1].compare("c") == 0 || cmds[1].compare("v") == 0){
            try{
              lexical_cast<int>(cmds[2]);
            }
            catch(boost::bad_lexical_cast&){
              if(cmds[2].compare("T") == 0){
                return true;
              }
              else{
                msg_="Invalid command:See list provided by the professor 2\n";
                std::size_t n = msg_.size();
                msg_.copy(error_buffer_, n);
                //write(sock_, boost::asio::buffer(error_buffer_,n));
                boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                      boost::bind(&connection::empty_handle_error, this, _1));
                return false;
              }
            }
            return true;
          }
          else if (cmds[1].compare("l") == 0 || cmds[1].compare("d") == 0 || cmds[1].compare("o") == 0 ||
                   cmds[1].compare("L") == 0 || cmds[1].compare("O") == 0 || cmds[1].compare("r") == 0){
            try{
              lexical_cast<int>(cmds[2]);
            }
            catch(boost::bad_lexical_cast&){
              msg_="Invalid command:See list provided by the professor\n";
              std::size_t n = msg_.size();
              msg_.copy(error_buffer_, n);
              //write(sock_, boost::asio::buffer(error_buffer_,n));
              boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                    boost::bind(&connection::empty_handle_error, this, _1));
              return false;
            }
            return true;
          }
          else{
            msg_="Invalid command:See list provided by the professor\n";
            std::size_t n = msg_.size();
            msg_.copy(error_buffer_, n);
            //write(sock_, boost::asio::buffer(error_buffer_,n));
            boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                  boost::bind(&connection::empty_handle_error, this, _1));
            return false;
          }
        }
        else{
          msg_="Invalid command:See list provided by the professor\n";
          std::size_t n = msg_.size();
          msg_.copy(error_buffer_, n);
          //write(sock_, boost::asio::buffer(error_buffer_,n));
          boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
                boost::bind(&connection::empty_handle_error, this, _1));
          return false;
        }
    }
    else{
      msg_="Invalid command: Wrong # of arguments\n";
      std::size_t n = msg_.size();
      msg_.copy(error_buffer_, n);
      //write(sock_, boost::asio::buffer(error_buffer_,n));
      boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
            boost::bind(&connection::empty_handle_error, this, _1));
      return false;
    }
  }
  else{
    msg_="Invalid command: Reset not implemented (For now, please use the reset button)\n";
    std::size_t n = msg_.size();
    msg_.copy(error_buffer_, n);
    //write(sock_, boost::asio::buffer(error_buffer_,n));
    boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),
          boost::bind(&connection::empty_handle_error, this, _1));
    return false;
  }
  return false;
}
void empty_handle_error(const boost::system::error_code& ec){
  if (ec){
    printf("Error when contacting client\n");
  }
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
      stopped_ = false;
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
