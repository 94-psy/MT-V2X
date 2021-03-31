// C library headers
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

//for soket
#include<arpa/inet.h>
#include<sys/socket.h>
#include <sys/time.h>
#include <sys/timerfd.h>    //for timer
#include <inttypes.h>       //for timer

#define SERVER "192.168.50.255"
#define PORT 8888	//The port on which to send data

void die(char *s)
{
	perror(s);
	exit(1);
}

void main(){

    struct sockaddr_in broadcastAddr;
    int s, l;
    unsigned int slen=sizeof(broadcastAddr);
    struct timespec timestamp;    //timestamp to add to payload
    double microsec = 0.0;
    char microsec_str[1000];
    int index = 0;

    //create a soket 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
            die("socket");
    }
    int broadcastEnable=1;
    int ret=setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));


    //clear buffer of soket
    memset((char *) &broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    inet_pton(AF_INET, "192.168.50.255", &broadcastAddr.sin_addr);
    broadcastAddr.sin_port = htons(PORT);

    while(index < 100){

        clock_gettime(CLOCK_REALTIME, &timestamp); //prendo il timestamp da mettere nel payload
        microsec = timestamp.tv_sec * 1000000LL + timestamp.tv_nsec / 1000;

        memset(&microsec_str, '\0', sizeof(microsec_str));
        snprintf(microsec_str, 1000, "%lf", microsec);

        if (sendto(s, microsec_str, strlen(microsec_str) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
            die("sendto()");
        }
          
        index++;
    }

    close(s);
}