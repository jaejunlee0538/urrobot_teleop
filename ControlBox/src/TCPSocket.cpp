//
// Created by ub1404 on 17. 2. 18.
//

#include "TCPSocket.h"

ssize_t TCPSocket::recvBytes(char* buffer, size_t len) {
  ssize_t ret = ::read(_sock_fd, buffer, len);
  if(ret == 0){
    //opponent socket is about to closed
    TCPSocket::close();
  }
  return ret;
}

ssize_t TCPSocket::sendBytes(const char* buffer, size_t len) {
  return ::write(_sock_fd, buffer, len);
}

void TCPSocket::close() {
  if(isConnected()) {
    ::close(_sock_fd);
    setConnected(false);
  }
}

bool TCPSocket::isReadyToReceive(uint32_t timeout_ms) {
  if(!isConnected()){
    return false;
  }
  fd_set reader;
  int fd_num;
  FD_ZERO(&reader);
  FD_SET(_sock_fd, &reader);
  timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms-timeout.tv_sec) * 1000;
  if((fd_num =::select(_sock_fd+1, &reader, NULL, NULL, &timeout)) == -1){
    return false;
  }
  if(FD_ISSET(_sock_fd, &reader)){
    return true;
  }
  return false;
}