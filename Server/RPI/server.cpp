//g++ -std=c++11 server.cpp -o server -lboost_system -pthread -lpigpio -lrt
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
std::string ser_port="/dev/ttyACM0";

bsc_xfer_t xfer;
#include "server_functions.cpp"
#define SCL 19
#define SDA 18
#define ADDRESS 0x09

int main(int argc, char* argv[]){
  int PORT=10000;
  try{
    if (argc != 2){
      std::cerr << "Default port used (PORT:10000)\n";
    }
    else{
      PORT=atoi(argv[1]);
    }

    boost::asio::io_service io;
    serial_connection *SPort= new serial_connection(io);
    SPort->start_serial_connection(ser_port,9600);
    initialize_i2c();
    thread t (i2c_read);
    server s(io, PORT, SPort);
    io.run();
  }
  catch (std::exception& e){
    std::cerr << "Exception: " << e.what() << "\n";
  }
  return 0;
}
