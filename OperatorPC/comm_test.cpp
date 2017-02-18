//
// Created by ub1404 on 17. 2. 17.
//

#include "TCPClient.h"
#include "URState.h"
#include "PacketManager.h"

int main(int argc, char** argv){
  TCPClient tcp_client;
  tcp_client.init();
  tcp_client.connect("127.0.0.1",50000);

  ::sleep(2);
  tcp_client.close();

  return 0;


}