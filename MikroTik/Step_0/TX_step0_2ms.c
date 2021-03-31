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

//baudrate
#define UART_SPEED B9600

//define ip brodcast for soket
//#define SERVER "192.168.1.5"
#define IPBRD "192.168.50.255"
#define BUFLEN 300	//Max length of buffer
#define PORT 8888	//The port on which to send data

//file pointer for log
FILE *fp ;
char file_name[] = "/root/802.11p/step_0/log_TX_step0.txt";
char UID[] = "2";

//char position[300] = "093437.10,4437.69452,N,01056.88114,E,0,,";

char position[300] = "093437.10,4437.66878,N,01056.93616,E,0,,";
char payload[300];

unsigned int sleep_time = 20000;
struct sockaddr_in broadcastAddr;     //soket brodcast
int s, l;
unsigned int slen=sizeof(broadcastAddr);
int lenght = 0;
int num_seq = 0;
char str_numseq[1000];
char buff_micro[100];
struct timespec timestamp;            //timestamp to add to payload
double microsec = 0.0; 


void reverse (char s[]);
void itoa(int n, char s[]);
void die(char *s);
void INThandler(int);

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

void die(char *s)
{
	perror(s);
	exit(1);
}

//---------------------------------------------------------//
//                CATCH CTRL+C
//--------------------------------------------------------//

void INThandler(int sig){

    signal(sig, SIG_IGN);
    printf("\n Quit \n");
    //kill the pending tasks
    //close file log, soket and serial port
    fclose(fp);
    close(s);
    exit(0);    
}

int main(){

    //create a soket 
    if ( (s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1){
            die("socket");
    }
    int broadcastEnable=1;
    int ret=setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));


    //clear buffer of soket
    memset((char *) &broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    inet_pton(AF_INET, IPBRD, &broadcastAddr.sin_addr);
    broadcastAddr.sin_port = htons(PORT);

    //open a log file and write a header
    fp=fopen(file_name,"a");
    fprintf(fp,"UTC,Latitude_raw,Latitude_dir,Longitude_raw,Longitude_dir,speed_raw,heading_raw,seq_num,UID,Timestamp\n");

    while(1){

        signal(SIGINT, INThandler);

        memset(&payload, '\0', sizeof(payload));
        memset(&str_numseq, '\0', sizeof(str_numseq));

        strcat(payload, position);
        num_seq++;
        itoa(num_seq, str_numseq);
        strcat(payload, str_numseq);
        strcat(payload, ",");
        strcat(payload, UID);
        strcat(payload, ",");

        //collect current time
        clock_gettime(CLOCK_REALTIME, &timestamp); //prendo il timestamp da mettere nel payload
        //timestamp conversion in microseconds
        microsec = timestamp.tv_sec * 1000000LL + timestamp.tv_nsec / 1000;

        //change format from double to string
        memset(&buff_micro, '\0', sizeof(buff_micro));
        snprintf(buff_micro, 100, "%lf", microsec);
        strcat(payload, buff_micro);
        strcat(payload, ",");


        //Dummy payload
        lenght = strlen(payload);

        if (lenght < 299){
            strcat(payload, ",-");
            int l;
            for (l=lenght+3; l<300; l++){
                strcat(payload, "0");
            }
        }

        //send the message
        if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
            die("sendto()");
        }

        lenght = strlen(payload);

        //write the log        
        fprintf(fp,"%s\n",payload);
        printf("Pyaload Sendt (%d Bytes) -> seq_num %d\n",lenght,num_seq);

        usleep(sleep_time);
    }
    return 0; // success
}