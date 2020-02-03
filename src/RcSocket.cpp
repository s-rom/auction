/*
  Library to manage Sockets
  Original Source Code: Ignasi Furio

  ArSocketLab.cc
  Creation Date : 18/08/2005
  Modifications: -
*/


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "RcSocket.h"

#ifndef INADDR_NONE
#define INADDR_NONE 0xffffffff
#endif

u_short portbase = 0;

//extern int errno;
//extern char *sys_errlist[]; 

int RcSocket::sock; 
pthread_mutex_t RcSocket::mutexSock;

RcSocket::RcSocket() {}

/*
------------------------------------------------------
  convert a string to  a IP address
------------------------------------------------------
*/
u_long RcSocket::inet_addr(char *hostIP) {
  char a1, a2, a3, a4;
  if (sscanf(hostIP, "%c.%c.%c.%c", &a1, &a2, &a3, &a4) == 4) {
    return a1 << 3 + a2 << 1 + a3 << 1 + a4;
  } else return INADDR_NONE;
}

/*--------------------------------------------------------------
*  connectsock - assigna i connecta un socket usant UDP o TCP  (CLIENT)
* ----------------------------------------------------------------
*/

int RcSocket::connectSocket(char *host, char *service, char *protocol) {
  struct hostent *phe; //punter a informacio del host
  struct servent *pse; //Servei associat al port
  struct protoent *ppe;
  struct sockaddr_in sin;
  int s, type;
  
  bzero((char *) &sin, sizeof(sin));
  sin.sin_family =  AF_INET;
  
  if (pse = getservbyname(service, protocol))
    sin.sin_port = pse->s_port;
  else if ((sin.sin_port = htons((u_short) atoi(service))) == 0)
    printf("No es possible accedir al servei \"%s\". \n", service);
  
  if (phe = gethostbyname(host))
    bcopy(phe->h_addr, (char *)&sin.sin_addr, phe->h_length);
  else if ((sin.sin_addr.s_addr = RcSocket::inet_addr(host)) == INADDR_NONE)
    printf("no es possible accedirt al host \"%s\". \n", host); 
  
          

  if ((ppe = getprotobyname(protocol)) == 0)
    printf("No es possible accedir al protocol \"%s\". \n", protocol);

  if (strcmp(protocol, "udp") == 0)
    type = SOCK_DGRAM;
  else
    type = SOCK_STREAM;
  
  s = socket(PF_INET, type, ppe->p_proto);
  if (s < 0)
    printf("No es pot crear el socket\n");

  if (connect(s, (struct sockaddr *)&sin, sizeof(sin)) < 0)
    printf("No es connecta a %s. %s\n", host, service);
  else
    printf("Socket Connected to %s.%s\n", host, service);
  return s;
} 

/*------------------------------------------------------------------
  passiveSocket - Crea un socket com a servidor
  qlen: longitut maxima de la cua del servidor
  --------------------------------------------------------------------
*/

int RcSocket::passiveSocket(char *service, char *protocol, int qlen) {
  struct servent *pse;
  struct protoent *ppe;
  struct sockaddr_in sin;
  int s, type;
  
  bzero((char *)&sin, sizeof(sin));
  sin.sin_family = AF_INET;
  sin.sin_addr.s_addr = INADDR_ANY;

  if (pse = getservbyname(service, protocol))
    sin.sin_port = htons(ntohs((u_short)pse->s_port)+portbase);
  else if ((sin.sin_port = htons((u_short) atoi(service))) == 0)
          printf("No es possible accedir al servei \"%s\". \n", service);

  if ((ppe = getprotobyname(protocol)) == 0)
    printf("No es possible accedir al protocol \"%s\". \n", protocol);

  if (strcmp(protocol, "udp") == 0)
    type = SOCK_DGRAM;
  else
    type = SOCK_STREAM;
  
  s = socket(PF_INET, type, ppe->p_proto);
  if (s < 0)
    printf("No es pot crear el socket\n");

  if (bind(s, (struct sockaddr *) &sin, sizeof(sin)) < 0) {
    printf("No es pot bloquejar el port %s \n", service);
    return (-1);
  }
  if ((type == SOCK_STREAM) && (listen(s, qlen) < 0)) {
    printf("No es pot esperar pel portr %s \n", service);
    return (-1);
  }

  return s;
}

int RcSocket::connectSocketUDP ()
{
	struct sockaddr_in address;
	int s;

	/* Se abre el socket UDP (DataGRAM) */
	
	if (s = socket(AF_INET, SOCK_DGRAM, 0) == (-1)) {
		return -1;
	}

	/* Se rellena la estructura de datos necesaria para hacer el bind() */
	address.sin_family = AF_INET;            /* Socket inet */
	address.sin_addr.s_addr = htonl(INADDR_ANY);    /* Cualquier direcci�n IP */
	address.sin_port = htons(0);                    /* Dejamos que linux eliga el servicio */

	/* Se hace el bind() */
	if (bind (s, (struct sockaddr *)&address, sizeof(s)) == -1) {
		//close(s);
		return -1;
	}

	/* Se devuelve el Descriptor */
	return s;
}


int RcSocket::fillAddress(char *host, char *port, struct sockaddr_in *sin)
{
  struct hostent *hp;
  if (sin == NULL) { printf("Sin es null\n"); return -1;}
  
  bzero(sin, sizeof(sin));
  sin->sin_family = AF_INET;
  //fsin.sin_addr.s_addr = RcSocket::inet_addr("127.0.0.1");
  hp = gethostbyname(host);

  if (hp == NULL) printf("hp es NULLL-->%i, %s\n", h_errno, host);
  /*while (hp == NULL) {
    hp = gethostbyname("localhost");
    printf("Try again\n");
  }*/
  //if (hp == NULL) printf("hp es NULLL2-->%i\n", h_errno);
  bcopy((char *)hp->h_addr, 
        (char *)&(sin->sin_addr),
         hp->h_length);
  sin->sin_port = htons(atoi(port)); 
  return 0;
}

int RcSocket::fillAddress(struct in_addr addr, unsigned short port, struct sockaddr_in *sin)
{
  struct hostent *hp;
  if (sin == NULL) return -1;
  bzero(sin, sizeof(sin));
  sin->sin_family = AF_INET;
  sin->sin_addr = addr;
  sin->sin_port = htons(port);  
  return 0;
}

bool RcSocket::sendMessage(char *message, char *host, char *port) {
  struct sockaddr_in fsin;
  //int sock= socket(AF_INET, SOCK_DGRAM, 0);
  RcSocket::fillAddress(host, port, &fsin);
  int n = sendto(RcSocket::sock, message, strlen(message), 0, (struct sockaddr*)&fsin, (socklen_t)sizeof(struct sockaddr_in));
  if (n < 0) {
    printf("Error Sendto \n");
    return false;
  }
  return true;
}

bool RcSocket::sendMessage(char *message, struct in_addr addr, unsigned short port) {
  struct sockaddr_in fsin;
  int n;
  //int sock= socket(AF_INET, SOCK_DGRAM, 0);

  RcSocket::fillAddress(addr,  port,  &fsin);
  
  pthread_mutex_lock(&RcSocket::mutexSock);
    n = sendto(RcSocket::sock, message, strlen(message), 0, (struct sockaddr*)&fsin, (socklen_t)sizeof(struct sockaddr_in));
  pthread_mutex_unlock(&RcSocket::mutexSock);
	 
  if (n < 0) {
    printf("Error Sendto \n");
    return false;
  }
  //close(sock);
  return true;
}

void RcSocket::initRcSocket() {
  RcSocket::sock= socket(AF_INET, SOCK_DGRAM, 0);
  pthread_mutex_init(&RcSocket::mutexSock, NULL);
}

