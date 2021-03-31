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
#include  <signal.h>    //handling CTRL+C

//for soket
#include<arpa/inet.h>
#include<sys/socket.h>
#include <sys/time.h>
#include <sys/timerfd.h>    //for timer
#include <inttypes.h>       //for timer

//semaphores and thread
#include <pthread.h>
#include <semaphore.h>

#define BUFLEN 300	//Max length of buffer
#define PORT 8888	//The port on which to send data
char UID[] = ",7";
char payload[BUFLEN];
unsigned int per_100 = 99500;   //per invio 100 ms

//variables semaphors
pthread_t t1;
pthread_attr_t myattr;

//variables soket
struct sockaddr_in broadcastAddr;
int s, l;
unsigned int slen=sizeof(broadcastAddr);

int number_thread = 0;

char payload_not_dummy[]="0,0,0,0,0,0,0,";
char num_seq_str[100];

void reverse (char s[]);
void INThandler(int);
void die(char *s);
void itoa(int n, char s[]);

//---------------------------------------------------------//
//                CATCH CTRL+C
//--------------------------------------------------------//

void INThandler(int sig){
    int i;

    signal(sig, SIG_IGN);
    printf("\n Quit \n");
    //kill the pending tasks
    for(i=0; i<number_thread; i++){
        pthread_attr_destroy(&myattr);
        pthread_cancel(t1);
        close(s);
    }
    exit(0);    
}

void die(char *s)
{
	perror(s);
	exit(1);
}

 /* reverse:  reverse string s in place */
void reverse(char s[]){
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}

void itoa(int n, char s[]){
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}

void *send_dummy(void *arg){


    while(1){

        memset(&payload, '\0', sizeof(payload));
        int num_seq = 0;
        int lenght = 0;
                
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

        //creo il payload + dummy
        memset(&payload, '\0', sizeof(payload));
        strcat(payload, payload_not_dummy);
        num_seq++;
        itoa(num_seq, num_seq_str);
        strcat(payload, num_seq_str);
        strcat(payload, UID);

        //creazione dummy payload
        lenght = strlen(payload);

        if (lenght < 299){
            //creo il dummy
            strcat(payload, ",-");
            int l;
            for (l=lenght+3; l<300; l++){
                strcat(payload, "7");
            }
        }

        printf("PAYLOAD : %s\n", payload);
        //send the message
        if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
            die("sendto()");
        }
            
        usleep(per_100);

    }

}

int main(void){

    int i;

	printf("Type in a number of thread disturbator \n");
	scanf("%d", &number_thread);

    signal(SIGINT, INThandler);
    int err;
    pthread_attr_init(&myattr);

    for(i=0; i<number_thread; i++){
        err = pthread_create(&t1, &myattr, send_dummy, (void *)"1");
    }
    pthread_attr_destroy(&myattr);

    pthread_exit(NULL); 

    printf("End process and thread\n");

    return 0;


}
