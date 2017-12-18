g++ -std=c++11 server_2.cpp -o server -lboost_system -pthread;
g++ -std=c++11 async_tcp_client.cpp -o client -lboost_system -pthread;
exit 0;
