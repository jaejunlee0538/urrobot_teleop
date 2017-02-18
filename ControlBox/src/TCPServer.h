//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_TCPSERVER_H
#define PLAYGROUND_TCPSERVER_H

#include "TCPSocket.h"
#include <string.h>
#include <stdio.h>

class TCPServer : public TCPSocket{
public:
  TCPServer():_serv_sock_fd(-1){

  }

  virtual ~TCPServer(){
    this->close();
  }

  void close(){
    //close client first
    TCPSocket::close();
    //close server socket
    if(isInitialized()){
      ::close(_serv_sock_fd);
      setInitialized(false);
    }
  }

  void closeClient(){
    TCPSocket::close();
  }

  int initialize(const int& port){
    if(isInitialized()){
      //already initialized.
      //first close socket
      this->close();
    }
    _serv_sock_fd = ::socket(PF_INET, SOCK_STREAM, 0);
    fprintf(stderr, "serv_sock_fd %d\n", _serv_sock_fd);
    memset(&_serv_addr, 0, sizeof(_serv_addr));
    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    _serv_addr.sin_port = htons(port);

    if(::bind(_serv_sock_fd, (struct sockaddr*)&_serv_addr, sizeof(_serv_addr)) == -1){
      return -1;
    }
    if(::listen(_serv_sock_fd, 1) == -1){
      return -1;
    }
    setInitialized(true);
    return 0;
  }

  bool waitForConnection(uint32_t timeout_ms){
    if(!isInitialized()){
      fprintf(stderr, "socket uninitialized\n");
      return false;
    }
    if(isConnected()){
      //Server has a connection already.
      fprintf(stderr, "already connected\n");
      return false;
    }
    fd_set conns;
    int fd_num;
    FD_ZERO(&conns);
    FD_SET(_serv_sock_fd, &conns);
    timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms-timeout.tv_sec) * 1000;
    fd_num = ::select(_serv_sock_fd+1, &conns, NULL, NULL, &timeout);
    if(fd_num == -1){
      fprintf(stderr, "select error\n");
      return false;
    }
    if(FD_ISSET(_serv_sock_fd, &conns)){
      socklen_t adr_sz;
      adr_sz = sizeof(_other_addr);
      _sock_fd = ::accept(_serv_sock_fd, (struct sockaddr*)&_other_addr, &adr_sz);
      if(_sock_fd == -1){
        fprintf(stderr, "accept error\n");
        return false;
      }
      setConnected(true);
      return true;
    }
    return false;
  }
private:
  int _serv_sock_fd;
  struct sockaddr_in _serv_addr;
};


#endif //PLAYGROUND_TCPSERVER_H
