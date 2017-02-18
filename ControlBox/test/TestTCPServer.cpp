#include <TCPServer.h>
#include <iostream>
int main(){
  TCPServer server;
  if(server.initialize(40000) == -1){
    std::cerr<<"Initialize error"<<std::endl;
    return 0;
  }

  while(!server.isConnected()){
    std::cout<<"Waiting for connection"<<std::endl;
    server.waitForConnection(500);
  }
  std::cout<<"Connected"<<std::endl;

  while(server.isConnected()){
    if(server.isReadyToReceive(5)){
      char buffer[1024];
      int len_rcv = server.recvBytes(buffer, 1024);
      std::cout<<len_rcv<<" bytes received"<<std::endl;
      if(len_rcv == 0){
        std::cout<<"Closing..."<<std::endl;
        server.close();
      }else if(len_rcv>0){
        buffer[len_rcv] = 0;
        std::cout<<"Received- : "<<buffer<<std::endl;
        server.sendBytes(buffer, len_rcv);
      }else{
        //error.
        std::cerr<<"Error while recvBytes"<<std::endl;
      }
    }
  }
  std::cout<<"Bye Bye~"<<std::endl;
  return 0;
}