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

//---------------------------------------------------------//
//                GLOBAL VARIBLES
//--------------------------------------------------------//

//variables semaphors
sem_t mysem;
pthread_t t1,t2;
pthread_attr_t myattr;

//variables soket
struct sockaddr_in si_me, si_other;
int s, slen = sizeof(si_other) , recv_len;

//variables for serial
int serial_port;
char port_name[] = "/dev/ttyACM0";

//varibles file log
FILE *fp_RX;
char perc_file[]="/root/802.11p/log_RX.txt";

char global_buffer_protected[300] = "\0"; //first don't know who gained acces--> may use priority
int num = 0;

//---------------------------------------------------------//
//                FUNCTIONS DECLARATIONS
//--------------------------------------------------------//

void INThandler(int);
void die(char *s);
double latency_IPG (double time1, double time2);
double F_distance (double lat1, double lat2, double long1, double long2, char *lat1_d, char *lat2_d, char *long1_d, char *long2_d);

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
    fclose(fp_RX);
    close(s);
    close(serial_port);
    exit(0);    
}

//---------------------------------------------------------//
//                FUNCTIONS FOR CALCULUS
//--------------------------------------------------------//

void die(char *s){

	perror(s);
	exit(1);
}

double F_distance (double lat1, double lat2, double long1, double long2, char *lat1_d, char *lat2_d, char *long1_d, char *long2_d){

  int result = -1;
  double R, a, c, d, lat_a, lat_b, long_a, long_b;
  R = 6371000.0; // Radius of the earth (in meters)

  double dd1, n_lat1, n_lat2, n_long1, n_long2;
  dd1 = (((int)(lat1/100))*100);  //4437 --> 4400
  n_lat1 = dd1/100 + ((lat1 - dd1)/60);   //nuova lat

  dd1 = (((int)(lat2/100))*100);  //4437 --> 4400
  n_lat2 = dd1/100 + ((lat2 - dd1)/60);   //nuova lat

  dd1 = (((int)(long1/100))*100);  //4437 --> 4400
  n_long1 = dd1/100 + ((long1 - dd1)/60);   //nuova longitudine

  dd1 = (((int)(long2/100))*100);  //4437 --> 4400
  n_long2 = dd1/100 + ((long2 - dd1)/60);   //nuova longitudine

  result = strcmp(lat1_d, "S");
  if (result == 0){
    n_lat1 = n_lat1 * -1;
  }

  result = strcmp(lat2_d, "S");
  if (result == 0){
    n_lat2 = n_lat2 * -1;
  }

  result = strcmp(long1_d, "W");
  if (result == 0){
    n_long1 = n_long1 * -1;
  }

  result = strcmp(long2_d, "W");
  if (result == 0){
    n_long2 = n_long2 * -1;
  }
  // Convert lat and long to radians
  lat_a = n_lat1 * (M_PI / 180.0);
  long_a = n_long1 * (M_PI / 180.0);
  lat_b = n_lat2 * (M_PI / 180.0);
  long_b = n_long2 * (M_PI / 180.0);
 
  // Apply the haversine formula
  a = pow((sin(lat_b - lat_a) / 2.0), 2) + (cos(lat_a) * cos(lat_b) * pow(sin(long_b - long_a) / 2.0, 2));
  c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  d = R * c;

  return d;
}

double latency_IPG (double time1, double time2){

  double diff = 0.0;

  diff=fabs(time1-time2); //difference in modules for float
  diff = diff/1000;

  return diff;
}

void *reader_SOKET(void *arg){

  //---------------------------------------------------------//
  //                   VARIABLES
  //--------------------------------------------------------//
    
  char buf[BUFLEN]; //buffer for soket data incoming

  //allocate memory for soket packet incoming
  char temp_string_soket[2000]; //for parsing SOKET data incoming
  char temp_arr_global_GPS[300];//for paste collected data from protected varible
  char temp_string_gps[20];     //into for GPS

  //counters for parsing
  int i,j,k;
  
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

  //all GSP data readed from global_GPS_protected
  char UTC_raw_gps[20];
  char connection_gps[2];
  char latitude_raw_gps[20];
  char longitude_raw_gps[20];
  char speed_raw_gps[10];
  char heading_raw_gps[10];
  char latitude_dir_gps[2];
  char longitude_dir_gps[2];

  char UID_exclude[2] = "7";  //exclude the Mtik Distrurb 

  //varibles for calculate distance/ipg/latency
  double UTC_RX_actual = 0.0;
  double UTC_RX_old = -1;
  double UTC_TX = 0.0;
  double latitude_TX = 0.0;
  double longitude_TX = 0.0;
  double latitude_RX = 0.0;
  double longitude_RX = 0.0;
  double IPG = 0.0;
  double Latency = 0.0;
  double Distance = 0.0;

  //variables for timer
  double start_timer = 0.0;
  double stop_timer = 0.0;
  struct timespec timestamp_RX;
  char timestamp_soket[10000];
  double microsec_TX = 0.0;
  double microsec_RX = 0.0;
    
  //---------------------------------------------------------//
  //                       SOKET
  //--------------------------------------------------------//

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

  //opern and write file header
  fp_RX=fopen(perc_file,"a");
  fprintf(fp_RX,"UTC_TX,Latitude_raw_TX,Latitude_dir_TX,Longitude_raw_TX,Longitude_dir_TX,speed_raw_TX,heading_raw_TX,seq_num_TX,UTC_RX,Latitude_raw_RX,Latitude_dir_RX,Longitude_raw_RX,Longitude_dir_RX,speed_raw_RX,heading_raw_RX,IPG,Latency,Distance_RXTX,UTC_RAW_GPS,diff,TimestampTX(us),Timestamp_RX(us),UID_TX\n");

  //---------------------------------------------------------//
  //           START WHILE FOR ALL OPERATIONS
  //--------------------------------------------------------//

  while(1){

    //catch the CTRL+C to end all tasks, close soket, close file, close serial port
    signal(SIGINT, INThandler);

    //clean the buffer
    memset(&buf, '\0', sizeof(buf));                                //clean the buffer of the soket
    memset(&temp_string_soket, '\0', sizeof(temp_string_soket));    //clean the temp buffer of soket (parsing)
    memset(&timestamp_soket, '\0', sizeof(timestamp_soket));        //clean the buffer of timestamp soket (math operation)
    memset(&temp_arr_global_GPS, '\0', sizeof(temp_arr_global_GPS));//clean the temp array thai containe the copy of protected varible

    //read from serial data incoming from TX
	  if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1){
		  die("recvfrom()");
	  }

    //if is not the disturbator Mtik
    if (buf[298] != UID_exclude[0]){

      //block semaphore to have mutual acces to the data of GPS (reader_GPS thread)
      sem_wait(&mysem);

      //copy the global varible into a temp array
      strcpy(temp_arr_global_GPS, global_buffer_protected);

      //free semaphore
      sem_post(&mysem);

      //collect current time for calculate Latency/IPG
      clock_gettime(CLOCK_REALTIME, &timestamp_RX);

      //parsing GPS data (temp_arr_global_GPS)
      i=0, j, k;

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

      //---------------------------------------------------------//
      //           CALCULATIONS SECTION
      //--------------------------------------------------------//
      
      microsec_TX = atof(timestamp_soket);
      UTC_RX_actual = 0.0;
      latitude_TX = atof(latitude_raw_soket);
      longitude_TX = atof(longitude_raw_soket);
      latitude_RX = atof(latitude_raw_gps);
      longitude_RX = atof(longitude_raw_gps);
      UTC_RX_actual = atof(UTC_raw_gps);
      UTC_TX = atof(UTC_raw_soket);

      Distance = F_distance(latitude_TX, latitude_RX, longitude_TX, longitude_RX, latitude_dir_soket, latitude_dir_gps, longitude_dir_soket, longitude_dir_gps);

      //timestamp conversion
      microsec_RX = timestamp_RX.tv_sec * 1000000LL + timestamp_RX.tv_nsec / 1000;

      Latency = latency_IPG(microsec_RX, microsec_TX);

      if(UTC_RX_old == -1){

        //first packet IPG=0
        UTC_RX_old = microsec_RX; 

      }

      IPG = latency_IPG(UTC_RX_old,microsec_RX);

      //for next cycle
      UTC_RX_old = microsec_RX;

      //write log
      fprintf(fp_RX,"%s,%s,%s,%s,%s,%s,%s,%s,%lf,%s,%s,%s,%s,%s,%s,%lf,%lf,%lf,%s,%lf,%lf,%s\n",UTC_raw_soket,latitude_raw_soket,latitude_dir_soket,longitude_raw_soket,longitude_dir_soket,speed_raw_soket,heading_raw_soket,seq_num_soket,UTC_RX_actual, latitude_raw_gps, latitude_dir_gps, longitude_raw_gps, longitude_dir_gps, speed_raw_gps, heading_raw_gps,IPG,Latency,Distance,UTC_raw_gps,microsec_TX,microsec_RX,UID);
      printf("num_seq: %s, IPG: %lf, Latency: %lf, Distance_RX_TX: %lf\n",seq_num_soket,IPG,Latency,Distance);
    }
    //end if disturbator
  }
  //end while
}

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

int main(){

  //hendler for CTRL+C
  signal(SIGINT, INThandler);
  int err;

  //init semaphore
  sem_init(&mysem,0,1);

  pthread_attr_init(&myattr);
  //create threads
  err = pthread_create(&t1, &myattr, reader_SOKET, (void *)"1");
  err = pthread_create(&t2, &myattr, reader_GPS, (void *)"2");
  pthread_attr_destroy(&myattr);
  
  //wait for pending threads
  pthread_join(t1, NULL);
  pthread_join(t2, NULL);

  pthread_exit(NULL); 

  printf("End process and thread\n");

  return 0;
}
