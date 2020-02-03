/*
   RcSocket.h
   Creation Date: 18/08/2005
   
*/
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>



#ifndef RC_SOCKET
#define RC_SOCKET


class RcSocket {
  public:
    RcSocket();
    static int connectSocket(char *host, char *service, char *protocol);
    static int passiveSocket(char *service, char *protocol, int qlen);
    static int connectSocketUDP();
    static int fillAddress(char *host, char *port, struct sockaddr_in *sin);
	 static int fillAddress(struct in_addr addr, unsigned short port, struct sockaddr_in *sin);
    static bool sendMessage(char *message, char *host, char *port);
	 static bool sendMessage(char *message, struct in_addr addr, unsigned short port);
 // private:
    static u_long inet_addr(char *hostIP);
	 static void initRcSocket();
	 
    static int sock;
    static pthread_mutex_t mutexSock;
};
#endif
