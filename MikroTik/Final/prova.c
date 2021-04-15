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

//baudrate
#define UART_SPEED B9600

//define ip brodcast for soket
//#define SERVER "192.168.1.5"
#define IPBRD "192.168.50.255"
#define BUFLEN 300	//Max length of buffer
#define PORT 8888	//The port on which to send data

//---------------------------------------------------------//
//                GLOBAL VARIBLES
//--------------------------------------------------------//

//curret id of mikrotik
int serial_port;
char UID[] = "2";
char port_name[] = "/dev/ttyACM0";

//trigger for CAM message
int trig_H = 0;     //trigger heading
int trig_TO = 0;    //trigger timeout
int trig_D = 0;     //trigger distance
int trig_S = 0;     //trigger speed

//file pointer for log
FILE *fp ;
char file_name[] = "/root/802.11p/log_TX.txt";

//variables semaphors
sem_t mysem;
pthread_t t1,t2;
pthread_attr_t myattr;

//variables soket
struct sockaddr_in broadcastAddr;     //soket brodcast
int s, l;
unsigned int slen=sizeof(broadcastAddr);
char buf[BUFLEN];
struct timespec timestamp;            //timestamp to add to payload
double microsec = 0.0;                //varible for paste current timestamp in microseconds

unsigned int sleep_time = 50000;

//variables for serial

char global_buffer_protected[300] = "\0"; //first don't know who gained acces--> may use priority
int num = 0;


//---------------------------------------------------------//
//                FUNCTIONS DECLARATIONS
//--------------------------------------------------------//
void reverse (char s[]);
void itoa(int n, char s[]);
void die(char *s);

//---------------------------------------------------------//
//                CATCH CTRL+C
//--------------------------------------------------------//

void INThandler(int sig){

    signal(sig, SIG_IGN);
    printf("\n Quit \n");
    //kill the pending tasks
    pthread_attr_destroy(&myattr);
    pthread_cancel(t1);
    pthread_cancel(t2);
    //close file log, soket and serial port
    fclose(fp);
    close(s);
    close(serial_port);
    exit(0);    
}

//---------------------------------------------------------//
//                FUNCTIONS FOR CALCULUS
//--------------------------------------------------------//

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
//                       SENDER SOKET
//--------------------------------------------------------//
void *sender_SOKET(void *arg){

    //allocate memory for soket packet incoming
    char temp_arr_global_GPS[300];//for paste collected data from protected varible
    char temp_string_gps[20];     //into for GPS

    //counters for parsing
    int i,j,k;

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


    //all GSP data readed from global_GPS_protected
    char UTC_raw_gps[20];
    char connection_gps[2];
    char latitude_raw_gps[20];
    char longitude_raw_gps[20];
    char speed_raw_gps[10];
    char heading_raw_gps[10];
    char latitude_dir_gps[2];
    char longitude_dir_gps[2];

    int lenght = 0;

    int num_seq = 0;
    char num_seq_str[10000];
    char payload[BUFLEN];
    char buff_micro[100];

    fp=fopen(file_name,"a");
    fprintf(fp,"UTC,Latitude_raw,Latitude_dir,Longitude_raw,Longitude_dir,speed_raw,heading_raw,seq_num,UID,Timestamp\n");

    while(1){

        //clean the buffer
        memset(&temp_arr_global_GPS, '\0', sizeof(temp_arr_global_GPS));//clean the temp array thai containe the copy of protected varible
        memset(&payload, '\0', sizeof(payload));
        //catch the CTRL+C to end all tasks, close soket, close file, close serial port
        signal(SIGINT, INThandler);

        //block semaphore to have mutual acces to the data of GPS (reader_GPS thread)
        sem_wait(&mysem);

        //copy the global varible into a temp array
        strcpy(temp_arr_global_GPS, global_buffer_protected);

        //free semaphore
        sem_post(&mysem);

        //parsing GPS data (temp_arr_global_GPS)
        i=0;

        for(j=0; j<9;j++){
            k=0;
            for (; temp_arr_global_GPS[i] != ','; i++){   
                temp_string_gps[k] = temp_arr_global_GPS[i];
                k++;
            }
            
            temp_string_gps[k] = '\0';

            if(j==0){
                //first is GPRMC
                i++;
            }
            if(j==1){
                i++;
                strcpy(UTC_raw_gps, temp_string_gps);
                //printf("UTC : %s\n", UTC_raw_gps);
            }
            if(j==2){
                i++;
                strcpy(connection_gps, temp_string_gps);
                //printf("connection : %s\n", connection);
            }
            if(j==3){
                i++;
                strcpy(latitude_raw_gps, temp_string_gps);
                //printf("latitude_raw : %s\n", latitude_raw);
            }
            if(j==4){
                i++;
                strcpy(latitude_dir_gps, temp_string_gps);
                //printf("latitude_dir : %s\n", latitude_dir);
            }
            if(j==5){
                i++;
                strcpy(longitude_raw_gps, temp_string_gps);
                //printf("longitude_raw : %s\n", longitude_raw);
            }
            if(j==6){
                i++;
                strcpy(longitude_dir_gps, temp_string_gps);
                //printf("longitude_dir : %s\n", longitude_dir);
            }
            if(j==7){
                i++;
                strcpy(speed_raw_gps, temp_string_gps);
                //printf("speed_raw : %s\n", speed_raw);
            }
            if(j==8){
                strcpy(heading_raw_gps, temp_string_gps);
                //printf("heading : %s\n", heading_raw);
            }
        }
        //payload creation

        strcat(payload, UTC_raw_gps);
        strcat(payload, ",");
        strcat(payload, latitude_raw_gps);
        strcat(payload, ",");
        strcat(payload, latitude_dir_gps);
        strcat(payload, ",");
        strcat(payload, longitude_raw_gps);
        strcat(payload, ",");
        strcat(payload, longitude_dir_gps);
        strcat(payload, ",");
        strcat(payload, speed_raw_gps);
        strcat(payload, ",");
        strcat(payload, heading_raw_gps);
        strcat(payload, ",");
        num_seq++;
        itoa(num_seq, num_seq_str);
        strcat(payload, num_seq_str);
        strcat(payload, ",");
        strcat(payload, UID);
        strcat(payload, ",");

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

        //write the log        
        fprintf(fp,"%s,%lf,%s,%lf,%s,%lf,%lf,%d,%s,%d,%d,%d,%d,%lf\n",UTC_raw_gps,latitude_raw_gps,latitude_dir_gps,longitude_raw_gps,longitude_dir_gps,speed_raw_gps,heading_raw_gps,num_seq,UID,microsec);
        printf("Pyaload Sendt (%d Bytes) -> seq_num %d %d\n", lenght, num_seq);

        usleep(sleep_time);

    }
    
}

//---------------------------------------------------------//
//                       READER GPS
//--------------------------------------------------------//
void *reader_GPS(void *arg){

  //---------------------------------------------------------//
  //                   VARIABLES
  //--------------------------------------------------------//
  char read_buf [300];

  //---------------------------------------------------------//
  //                   SERIAL CREATE/OPEN
  //--------------------------------------------------------//

  // Open the serial port
  serial_port = open(port_name,  O_RDWR );
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate
  cfsetispeed(&tty, UART_SPEED);
  cfsetospeed(&tty, UART_SPEED);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  while(1){

    //all functions for reading and parsing from serial

    signal(SIGINT, INThandler);

    memset(&read_buf, '\0', sizeof(read_buf));          //clean the buffer of serial port
    read(serial_port, &read_buf, sizeof(read_buf));

    while(read_buf[3] != 'R'){
      //Read correct data (== R --> GPRMC) from GPS USB 
      tcflush(serial_port, TCIFLUSH);                 //clean the USB internal buffer
      read(serial_port, &read_buf, sizeof(read_buf)); //read the serial port
    }

    //block semaphore to have mutual acces to the data of global_buffer_protected
    sem_wait(&mysem);
  
    //copy raw GPS data into global_buffer_protected
    strcpy(global_buffer_protected, read_buf);

    //leave the semaphore for other task
    sem_post(&mysem);
  }

}

//---------------------------------------------------------//
//                       MAIN
//--------------------------------------------------------//

int main (){

//hendler for CTRL+C
  signal(SIGINT, INThandler);
  int err;

  //init semaphore
  sem_init(&mysem,0,1);

  pthread_attr_init(&myattr);
  //create threads
  err = pthread_create(&t1, &myattr, sender_SOKET, (void *)"1");
  err = pthread_create(&t2, &myattr, reader_GPS, (void *)"2");
  pthread_attr_destroy(&myattr);
  
  //wait for pending threads
  pthread_join(t1, NULL);
  pthread_join(t2, NULL);

  pthread_exit(NULL); 

  printf("End process and thread\n");

  return 0;
}
