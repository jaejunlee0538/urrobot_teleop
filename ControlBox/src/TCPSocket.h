//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_TCPSOCKET_H
#define PLAYGROUND_TCPSOCKET_H
#include "SocketInterface.h"
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <arpa/inet.h>

class TCPSocket : public SocketInterface{
public:
  virtual ~TCPSocket(){}

  virtual void close();

  ssize_t sendBytes(const char* buffer, size_t len);

  ssize_t recvBytes(char* buffer, size_t len);

  bool isReadyToReceive(uint32_t timeout_ms);
protected:
  int _sock_fd;
};


#endif //PLAYGROUND_TCPSOCKET_H
