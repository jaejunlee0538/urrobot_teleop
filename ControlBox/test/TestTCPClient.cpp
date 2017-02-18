#include <TCPClient.h>
#include <iostream>
int main(){
  TCPClient client;
  if(client.initialize(40000) == -1){
    std::cerr<<"Initialize error"<<std::endl;
    return 0;
  }

  std::cout<<"Connecting..."<<std::endl;
  if(client.connect("127.0.0.1") == -1){
    std::cerr<<"Cannot connect to server"<<std::endl;
    return 0;
  }
  std::cout<<"Connected"<<std::endl;

  char str_buffer[1024];
  while(client.isConnected()){
    str_buffer[0] = 0;
    fgets(str_buffer, 1024, stdin);
    if(strcmp(str_buffer,"q\n")==0 || strcmp(str_buffer, "Q\n")==0){
      client.close();
      break;
    }

    str_buffer[strlen(str_buffer)-1] = 0;
    std::cout<<str_buffer<<std::endl;
    client.sendBytes(str_buffer, strlen(str_buffer));

    if(client.isReadyToReceive(1000)){
      int len_rcv = client.recvBytes(str_buffer, 1024);
      if(len_rcv == 0){
        std::cout<<"Closing..."<<std::endl;
        client.close();
      }else if(len_rcv>0){
        str_buffer[len_rcv] = 0;
        std::cout<<"Received : "<<str_buffer<<std::endl;
      }else{
        //error.
        std::cerr<<"Error while recvBytes"<<std::endl;
      }
    }else{
      std::cerr<<"Recv Timeout!"<<std::endl;
    }
  }
  std::cout<<"Bye Bye~"<<std::endl;
  return 0;
}