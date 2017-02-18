//
// Created by ub1404 on 17. 2. 18.
//

#ifndef PLAYGROUND_SOCKETINTERFACE_H
#define PLAYGROUND_SOCKETINTERFACE_H
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
class SocketInterface {
public:
  SocketInterface(){
    setConnected(false);
    setInitialized(false);
  }
  virtual ~SocketInterface(){}

  /*
   * send data of len bytes length to the opponent.
   * return :
   *    length(bytes) of data sent , when it successfully sent data
   */
  virtual ssize_t sendBytes(const char* buffer, size_t len) = 0;

  /*
   * read received data from socket buffer.
   * return :
   *    length(bytes) of data read, when it successfully read data
   *    0, if the opponent socket is about to be closed
   *    -1, when an error occurs
   */
  virtual ssize_t recvBytes(char* buffer, size_t len) = 0;

  /*
   * return true,
   *    if there is data to be read in socket buffer
   *    or opponent socket is about to closed(the following recvBytes call will return 0)
   * return false,
   *    otherwise
   */
  virtual bool isReadyToReceive(uint32_t timeout_ms) = 0;
  virtual void close() = 0;
  virtual int initialize(const int& port) = 0;
  const char* getOtherIP() const{
    if(!isConnected()){
      return "Not connected";
    }
    return inet_ntoa(_other_addr.sin_addr);
  }
  const int& getPort() const{
    if(!isConnected()){
      return -1;
    }
    return _other_addr.sin_port;
  }
  static const char* getMyIP();
  bool isInitialized() const{return _initialized;}
  bool isConnected() const{return _connected;}
protected:
  void setConnected(bool connected){
    _connected = connected;
  }
  void setInitialized(bool initialized){
    _initialized = initialized;
  }

  /*
   * Address of opponent with which communication is established.
   */
  struct sockaddr_in _other_addr;
private:
  bool _connected;
  bool _initialized;
};


#endif //PLAYGROUND_SOCKETINTERFACE_H
