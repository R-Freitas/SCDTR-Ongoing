#include "server_classes.hpp"


    serial_connection::serial_connection(boost::asio::io_service& io)
    : SPort(io)
    {
    }

    void serial_connection::start_serial_connection(std::string port, unsigned int baud_rate){
      boost::system::error_code error;
      SPort.open(port, error); //connect to port containing arduino
      if( error ){
          cout << "Error when connecting to Arduino \n";
          return ;
      }
      SPort.set_option(serial_port_base::baud_rate(baud_rate),error);
      if( error ) {
          cout << "Error when initializing connection to Arduino \n";
          return ;
      }
    }

    //Function to schedule the process of serial communication with the Arduino.
    void serial_connection::serial_send(std::string line){
      std::string terminated_line;
      std::cout << "Sending: " << line << "\n";
      terminated_line = line + std::string("\n");
      std::size_t n = terminated_line.size();
      terminated_line.copy(send_buffer_, n);
      boost::asio::async_write(SPort, boost::asio::buffer(send_buffer_,n),boost::bind(&serial_connection::handle_serial_send, this, _1));
    }


    //Deals with possible errors from the serial communication.
    void serial_connection::handle_serial_send(const boost::system::error_code& ec){
      if (ec)
      {
        std::cout << "Error on serial communication to Arduino\n";
      }
    }




//Class to manage the connection between client and server.
//Handles the various assinchronous processes between client and
//Arduino communication.
      socket_connection::socket_connection(boost::asio::io_service& io)
        : sock_(io),
          KeepAlive_(io),
          SPort(io)
      {
      }

      tcp::socket& socket_connection::socket(){
        return sock_;
      }
      //Functions to manage the server. Start it, end a connection with a client
      //and kill the server. These functions are called acording to user commands
      //from the client.
      //Iniciates the various processes
      void socket_connection::start_socket_connection(){
        stopped_ = false;
        KeepAlive_.expires_from_now(boost::posix_time::seconds(28));
        KeepAlive_.async_wait(boost::bind(&socket_connection::start_KeepAlive, this));
        SPort.start_serial_connection("/dev/ttyACM0",9600);
        start_read_socket_connection();
      }


      //Stops the server and waits for another connection
      void socket_connection::stop_socket_connection(){
        stopped_ = true;
        boost::system::error_code ignored_ec;
        sock_.close(ignored_ec);
        KeepAlive_.cancel();
      }


      //Shuts down the server, ending the various processes.
      void socket_connection::kill_server(){
        stopped_ = true;
        boost::system::error_code ignored_ec;
        sock_.close(ignored_ec);
        KeepAlive_.cancel();
        exit(0);
      }




      //Function to start the timer to send an heartbeat signal to the client.
      void socket_connection::start_KeepAlive(){
        if (stopped_){
          return;
        }
        else{
            socket_connection::send_socket_connection("\n",true);
        }
      }


      //Function that resets the timer eaxh time a heartbeat signal is sent.
      void socket_connection::handle_KeepAlive(const boost::system::error_code& ec){
        if (stopped_){
          return;
        }
        if (!ec){
          KeepAlive_.expires_from_now(boost::posix_time::seconds(28));
          KeepAlive_.async_wait(boost::bind(&socket_connection::start_KeepAlive, this));
        }
        else{
          std::cout << "Client disconnected" << "\n";
          stop_socket_connection();
        }
      }


      //Function that reads from the socket connection.
      void socket_connection::start_read_socket_connection(){
        boost::asio::async_read_until(sock_, input_buffer_, '\n',boost::bind(&socket_connection::handle_read_socket_connection, this, _1));
      }


      //Function to handle possible errors from the process of reading the socket.
      void socket_connection::handle_read_socket_connection(const boost::system::error_code& ec){
        if (stopped_){
            return;
        }

        if (!ec){
          // If no error occurred, extract the newline-delimited message from the buffer.
          std::string line;
          std::istream is(&input_buffer_);
          std::getline(is, line);

          //Ignore empty heartbeat messages.
          if (!line.empty()){
            //Check to see if a valid command was used.
            if (is_command_valid(line)){
              SPort.serial_send(line);
            }
          }
        }
        socket_connection::start_read_socket_connection();
      }

      void socket_connection::send_socket_connection(std::string msg, bool is_heartbeat){
        enum {max_length=1024};
        char error_buffer_[max_length];
        std::size_t n = msg.size();
        msg.copy(error_buffer_, n);
        boost::asio::async_write(sock_, boost::asio::buffer(error_buffer_,n),boost::bind(&socket_connection::handle_send_socket_connection, this, _1, is_heartbeat));
      }




      //Function to handle possible errors from sending a message to the client
      //through the socket. If a connection was successfull, the heartbeat
      //timer is reset to avoid sending useless heartbeat messages.
      void socket_connection::handle_send_socket_connection(const boost::system::error_code& ec, bool is_heartbeat){
        if (ec){
          printf("Erro a enviar mensagem de erro para o client\n");
        }
        else{
          if (is_heartbeat){
            socket_connection::handle_KeepAlive(ec);
          }
          else{
            KeepAlive_.expires_from_now(boost::posix_time::seconds(28));
          }
        }
      }


      //Function that performs various tests to ensure a valid command was used.
      bool socket_connection::is_command_valid(std::string &new_line){

        std::vector<std::string> cmds;
        std::string msg_;
        enum {max_length=1024};
        char error_buffer_[max_length];

        if (new_line.compare("KILL")!=0){
          if (new_line.compare("EXIT")!=0){
              if(new_line.compare("r")!=0){
                split(cmds, new_line, is_any_of(" ")); // here it is
                if (cmds.size()==3){
                    if (cmds[0].compare("s")== 0){
                      try{
                        int ocup = lexical_cast<int>(cmds[2]);
                        lexical_cast<int>(cmds[1]);
                        if (ocup!=0 && ocup!=1){
                          msg_="Invalid command:Occupancy command is used as 's <desk_i> <val>'\n";
                          send_socket_connection(msg_, false);
                          return false;
                        }
                      }
                      catch(boost::bad_lexical_cast&){
                        msg_="Invalid command:Occupancy command is used as 's <desk_i> <val>'\n";
                        send_socket_connection(msg_, false);
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
                          send_socket_connection(msg_, false);
                          return false;
                        }
                        return true;
                      }
                      else{
                        msg_="Invalid command:See list provided by the professor\n";
                        send_socket_connection(msg_, false);
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
                            send_socket_connection(msg_, false);
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
                          send_socket_connection(msg_, false);
                          return false;
                        }
                        return true;
                      }
                      else{
                        msg_="Invalid command:See list provided by the professor\n";
                        send_socket_connection(msg_, false);
                        return false;
                      }
                    }
                    else{
                      msg_="Invalid command:See list provided by the professor\n";
                      send_socket_connection(msg_, false);
                      return false;
                    }
                }
                else{
                  msg_="Invalid command: Wrong # of arguments\n";
                  send_socket_connection(msg_, false);
                  return false;
                }
              }
              else{
                msg_="Invalid command: Reset not implemented (For now, please use the reset button)\n";
                send_socket_connection(msg_, false);
                return false;
              }
          }
          else{
              msg_="\n Client disconnected\n";
              send_socket_connection(msg_, false);
              stop_socket_connection();
              return false;
          }
          return false;
        }
        else{
          msg_="Server killed\n";
          send_socket_connection(msg_, false);
          kill_server();
          return false;
        }
    }


//Class to enclose the various processes used in initializing a server.

      server::server(boost::asio::io_service& io_service, short port)
        : io_service_(io_service),
          acceptor_(io_service, tcp::endpoint(tcp::v4(), port))
      {
        start_accept();
      }


      void server::start_accept(){
        socket_connection* new_socket_connection = new socket_connection(io_service_);
        acceptor_.async_accept(new_socket_connection->socket(),
            boost::bind(&server::handle_accept, this, new_socket_connection,
              boost::asio::placeholders::error));
      }

      void server::handle_accept(socket_connection* new_socket_connection, const boost::system::error_code& ec){
        if (!ec){
          new_socket_connection->start_socket_connection();
          stopped_ = false;
          std::cout << "New client connected\n";
        }
        else{
          delete new_socket_connection;
        }
        server::start_accept();
      }


      /*
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
        printf("HEY2\n");
      }
      */
