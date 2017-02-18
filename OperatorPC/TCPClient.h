//
// Created by ub1404 on 15. 9. 18.
//

#ifndef PLAYGROUND_TCPCLIENT_H
#define PLAYGROUND_TCPCLIENT_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/select.h>
#include <pthread.h>
#include <errno.h>

class TCPClient{
public:
    TCPClient(){
        pthread_mutex_init(&this->mutex, NULL);
        this->is_connected = false;
        this->sock = -1;
    }

    ~TCPClient()
    {
        pthread_mutex_destroy(&this->mutex);
        this->close();
    }

    void close()
    {
        if(this->sock != -1){
            ::close(this->sock);
            this->setConnected(false);
            this->sock = -1;
        }
    }

    int init()
    {
        this->close();
        this->sock = socket(PF_INET, SOCK_STREAM, 0);
        if(this->sock == -1){
            return -1;
        }
        return 1;
    }

    int connect(const char* server_ip, int port)
    {
        memset(&this->serv_addr, 0, sizeof(this->serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = inet_addr(server_ip);
        serv_addr.sin_port = htons(port);
        if(::connect(this->sock, (struct sockaddr*)&this->serv_addr, sizeof(this->serv_addr)) == -1){
            printf("Connect failed\n");
            this->close();
            return -1;
        }

        this->setConnected(true);
        return 1;
    }

    bool isConnected(){
        pthread_mutex_lock(&this->mutex);
        bool ret = this->is_connected;
        pthread_mutex_unlock(&this->mutex);
        return ret;
    }

    void setConnected(bool connected){
        if(connected)
            printf("Connection established\n");
        else
            printf("Connection destroied\n");
        pthread_mutex_lock(&this->mutex);
        this->is_connected = connected;
        pthread_mutex_unlock(&this->mutex);
    }

    ssize_t write(const void * buf, size_t n)
    {
        if(!this->isConnected())
            return -1;
        return ::write(this->sock, buf, n);
    }

    ssize_t read(void * buf, size_t nbytes)
    {
        if(!this->isConnected())
            return -1;
        ssize_t read_len = ::read(this->sock, buf, nbytes);
        if(read_len < 0)
            return read_len;

        if(read_len == 0){
            //disconnect
            this->setConnected(false);
            ::close(sock);
            sock = -1;
            return 0;
        }
        return read_len;
    }

    void printErrorCode(){
        printf("Error : %d\n", errno);
    }
protected:
    int sock;
    struct sockaddr_in serv_addr;
    bool is_connected;
    pthread_mutex_t mutex;
};
#endif //PLAYGROUND_TCPCLIENT_H
