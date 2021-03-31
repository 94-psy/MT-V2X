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
#define BUFLEN 300	//Max length of buffer

void die(char *s)
{
	perror(s);
	exit(1);
}

double F_latency (double time1, double time2){

    double diff = 0.0;

    diff=fabs(time1-time2); //difference in modules for float
    diff = diff;

    return diff;
}

void main(){

    struct sockaddr_in si_me, si_other;
        
    int s, i, slen = sizeof(si_other) , recv_len;
    char buf[BUFLEN];
    double microsec_TX = 0.0;
    double microsec_RX = 0.0;
    struct timespec timestamp;    //timestamp to add to payload
    double latency = 0.0;
    int index = 0;
        
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
        die("socket");
    }

    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
        
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);
        
    //bind socket to port
    if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1){
        die("bind");
    }

    while(index < 100){
        
        memset(&buf, '\0', sizeof(buf)); 
        //lettura da soket dei dati in arrivo da TX
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1){
            die("recvfrom()");
        }

        clock_gettime(CLOCK_REALTIME, &timestamp); //prendo il timestamp da mettere nel payload
        microsec_RX = timestamp.tv_sec * 1000000LL + timestamp.tv_nsec / 1000;

        microsec_TX = atof(buf);

        latency = F_latency (microsec_RX, microsec_TX);
        latency = latency /1000;

        printf("Latenza in ms : %lf\n", latency);
    
        index++;
    }
}