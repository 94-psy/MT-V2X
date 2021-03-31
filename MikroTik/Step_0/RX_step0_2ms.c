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

#define UART_SPEED B9600
#define BUFLEN 300	//Max length of buffer
#define PORT 8888	//The port on which to listen for incoming data

//file pointer for log
FILE *fp ;
char file_name[] = "/root/802.11p/step_0/log_RX_step0.txt";
char UID[] = "3";

char position[300] = "093437.10,4437.69452,N,01056.88114,E,0,,";

char payload[300];

//variables soket
struct sockaddr_in si_me, si_other;
int s, slen = sizeof(si_other) , recv_len;
char buf[BUFLEN]; //buffer for soket data incoming
struct timespec timestamp_RX;
char timestamp_soket[10000];

//allocate memory for soket packet incoming
char temp_string_soket[2000]; //for parsing SOKET data incoming

//varibles for calculate distance/ipg/latency
  //all SOKET data collected
char UTC_raw_soket[20];
char latitude_raw_soket[20];
char longitude_raw_soket[20];
char speed_raw_soket[10];
char heading_raw_soket[10];
char latitude_dir_soket[2];
char longitude_dir_soket[2];
char seq_num_soket[10];
char UID[2];
double IPG = 0.0;
double Latency = 0.0;
double microsec_TX = 0.0;
double microsec_RX = 0.0;
double microsec_RX_old = -1;
char distance_RXTX[20] = "40";


void reverse (char s[]);
void itoa(int n, char s[]);
void die(char *s);
double latency_IPG (double time1, double time2);

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

double latency_IPG (double time1, double time2){

  double diff = 0.0;

  diff=fabs(time1-time2); //difference in modules for float
  diff = diff/1000;

  return diff;
}

int main(){

    int i,j,k;

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

    fp=fopen(file_name,"a");
    fprintf(fp,"UTC_TX,Latitude_raw_TX,Latitude_dir_TX,Longitude_raw_TX,Longitude_dir_TX,speed_raw_TX,heading_raw_TX,seq_num_TX,UTC_RX,Latitude_raw_RX,Latitude_dir_RX,Longitude_raw_RX,Longitude_dir_RX,speed_raw_RX,heading_raw_RX,IPG,Latency,Distance_RXTX,UTC_RAW_GPS,diff,TimestampTX,Timestamp_RX,UID_TX\n");



    while(1){

      memset(&buf, '\0', sizeof(buf));                                //clean the buffer of the soket
      memset(&temp_string_soket, '\0', sizeof(temp_string_soket));  

      //read from serial data incoming from TX
	  if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1){
		  die("recvfrom()");
	  }

      clock_gettime(CLOCK_REALTIME, &timestamp_RX);

      //parsing SOKET (buf)
      i=0, k=0, j=0;

      for(j=0; j<10;j++){ //j = numbers of data that i want to collect
        k=0;
        for (; buf[i] != ','; i++){

          temp_string_soket[k] = buf[i];
          k++;

        }
          
        temp_string_soket[k] = '\0';  //convert data into string

        if(j==0){
          i++;
          strcpy(UTC_raw_soket, temp_string_soket);
        }
        if(j==1){
          i++;
          strcpy(latitude_raw_soket, temp_string_soket);
          //printf("UTC : %s\n", UTC_raw);
        }
        if(j==2){                
          i++;
          strcpy(latitude_dir_soket, temp_string_soket);
          //printf("connection : %s\n", connection);
        }
        if(j==3){
          i++;
          strcpy(longitude_raw_soket, temp_string_soket);
          //printf("latitude_raw : %s\n", latitude_raw);
        }
        if(j==4){
          i++;
          strcpy(longitude_dir_soket, temp_string_soket);
          //printf("latitude_dir : %s\n", latitude_dir);
        }
        if(j==5){
          i++;
          strcpy(speed_raw_soket, temp_string_soket);
          //printf("longitude_raw : %s\n", longitude_raw);
        }
        if(j==6){
          i++;
          strcpy(heading_raw_soket, temp_string_soket);
          //printf("longitude_dir : %s\n", longitude_dir);
        }
        if(j==7){
          i++;
          strcpy(seq_num_soket, temp_string_soket);
          //printf("speed_raw : %s\n", speed_raw);
        }
        if(j==8){
          i++;
          strcpy(UID, temp_string_soket);
          //printf("speed_raw : %s\n", speed_raw);
        }
        if(j==9){
          i++;
          strcpy(timestamp_soket, temp_string_soket);
          //printf("speed_raw : %s\n", speed_raw);
        }
      }

      microsec_TX = atof(timestamp_soket);
      //timestamp conversion
      microsec_RX = timestamp_RX.tv_sec * 1000000LL + timestamp_RX.tv_nsec / 1000;

      Latency = latency_IPG(microsec_RX, microsec_TX);

      if (microsec_RX_old == -1){
            //first packet IPG=0
            microsec_RX_old = microsec_RX; 
      }
      
      IPG = latency_IPG(microsec_RX_old,microsec_RX);
      microsec_RX_old = microsec_RX;

      double UTC_RX_actual = microsec_RX; 
      char latitude_raw_gps [20] = "4433.69452";
      char latitude_dir_gps[20] = "N";
      char longitude_raw_gps[20] = "01056.88118";
      char longitude_dir_gps[2] = "E";
      char speed_raw_gps[20] = "0";
      char heading_raw_gps[20] = "0";
      double Distance = 80;
      char UTC_raw_gps[20] = "093437.10";

      //write log
      fprintf(fp,"%s,%s,%s,%s,%s,%s,%s,%s,%lf,%s,%s,%s,%s,%s,%s,%lf,%lf,%lf,%s,%lf,%lf,%s\n",UTC_raw_soket,latitude_raw_soket,latitude_dir_soket,longitude_raw_soket,longitude_dir_soket,speed_raw_soket,heading_raw_soket,seq_num_soket,UTC_RX_actual, latitude_raw_gps, latitude_dir_gps, longitude_raw_gps, longitude_dir_gps, speed_raw_gps, heading_raw_gps,IPG,Latency,Distance,UTC_raw_gps,microsec_TX,microsec_RX,UID);
      printf("num_seq: %s, IPG: %lf, Latency: %lf\n",seq_num_soket,IPG,Latency);
                  
    }
return 0;
}