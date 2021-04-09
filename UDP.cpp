#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


#include <string.h>
#include <iostream>
#include <unistd.h>



//Wait indefinitely until getting a UDP message, then fill UDP_Received with contents of message
int UDP_Recv(char * UDP_Received){


    //UDP socket setup
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(8888);

    bind(sock, (struct sockaddr*) &addr, sizeof(addr));

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));



    //Loop until data is received
    while (1) {

        char msgbuf[256];
        socklen_t addrlen = sizeof(addr);
        int nbytes = recvfrom(sock,msgbuf,256,0,(struct sockaddr *) &addr, (socklen_t*)&addrlen);

        if (nbytes > 0) {
          memcpy(UDP_Received, &msgbuf, 256);
          close(sock);
          break;
        }


     }


    return 0;

}


int main()
{


    while(1){

      

      char a[256];

      UDP_Recv(a);


      std::cout << a << std::endl;

    }
    
}