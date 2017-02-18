//
// Created by ub1404 on 15. 9. 17.
//

#ifndef PLAYGROUND_TCPSOCKET_H
#define PLAYGROUND_TCPSOCKET_H
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
#define UR_CAPI_PORT    30005

class TCPServer {
public:
    TCPServer(){
        this->serv_sock = -1;
        this->clnt_sock = -1;
        this->is_connected = false;
        pthread_mutex_init(&this->mutex, NULL);
    }

    ~TCPServer()
    {
        pthread_mutex_destroy(&this->mutex);
        this->close();
    }

    void close()
    {
        this->closeClient();
        if(this->serv_sock != -1){
            ::close(this->serv_sock);
            this->serv_sock = -1;
        }
    }

    void closeClient(){
        if(this->clnt_sock != -1){
            ::close(this->clnt_sock);
            this->clnt_sock = -1;
            this->setConnected(false);
        }
    }

    int init(int port)
    {
        this->close();
        this->serv_sock = socket(PF_INET, SOCK_STREAM, 0);
        if(this->serv_sock < 0){
            printf("Socket error\n");
            return -1;
        }
        memset(&this->serv_addr, 0, sizeof(this->serv_addr));
        this->serv_addr.sin_family = AF_INET;
        this->serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        this->serv_addr.sin_port=htons(port);

        if(::bind(this->serv_sock, (struct sockaddr*)&this->serv_addr, sizeof(this->serv_addr)) == -1){
            printf("bind error\n");
            this->close();
            return -1;
        }

        if(::listen(this->serv_sock, 5) == -1){
            printf("listen error\n");
            this->close();
            return -1;
        }
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

    int waitForConnection(void)
    {
        if(this->clnt_sock != -1){
            return -1;
        }
        this->clnt_sock = ::accept(this->serv_sock, (struct sockaddr*)&this->clnt_addr, &this->clnt_addr_size);

        if(clnt_sock == -1){
            return -1;
        }
        else{
            this->setConnected(true);
        }
        return 0;
    }

    ssize_t write(const void * buf, size_t n)
    {
        if(!this->isConnected())
            return -1;
        return ::write(this->clnt_sock, buf, n);
    }

    ssize_t read(void * buf, size_t nbytes)
    {
        if(!this->isConnected())
            return -1;
        ssize_t read_len = ::read(this->clnt_sock, buf, nbytes);
        if(read_len < 0)
            return read_len;

        if(read_len == 0){
            //disconnect
            this->setConnected(false);
            ::close(clnt_sock);
            clnt_sock = -1;
            return -1;
        }
        else{

        }
        return read_len;
    }

    void printErrorCode(){
        printf("Error : %d\n", errno);
    }

protected:
    int serv_sock;
    int clnt_sock;

    struct sockaddr_in serv_addr;
    struct sockaddr_in clnt_addr;
    socklen_t clnt_addr_size;

    bool is_connected;
    pthread_mutex_t mutex;
};


#endif //PLAYGROUND_TCPSOCKET_H
