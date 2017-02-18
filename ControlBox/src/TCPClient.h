//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_TCPCLIENT_H
#define PLAYGROUND_TCPCLIENT_H
#include "TCPSocket.h"
#include <string.h>

class TCPClient : public TCPSocket{
public:
  virtual ~TCPClient(){
    this->close();
  }

  void close(){
    TCPSocket::close();
    setInitialized(false);
  }

  int initialize(const int& port){
    if(isInitialized()){
      this->close();
    }
    _sock_fd = ::socket(PF_INET, SOCK_STREAM, 0);
    if(_sock_fd == -1){
      return false;
    }
    _port = port;
    setInitialized(true);
    return 0;
  }

  int connect(const char* ip_address){
    ::memset(&_other_addr, 0, sizeof(_other_addr));
    _other_addr.sin_family = AF_INET;
    _other_addr.sin_addr.s_addr = inet_addr(ip_address);
    _other_addr.sin_port = htons(_port);
    if(::connect(_sock_fd, (struct sockaddr*)&_other_addr, sizeof(_other_addr)) == -1){
      return -1;
    }
    setConnected(true);
    return 0;
  }
protected:
  int _port;
};


#endif //PLAYGROUND_TCPCLIENT_H
