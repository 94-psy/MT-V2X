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

//baudrate
#define UART_SPEED B9600
#define BUFLEN 300	//Max length of buffer

//---------------------------------------------------------//
//                GLOBAL VARIBLES
//--------------------------------------------------------//

//curret id of mikrotik
char UID[] = "1";
char port_name[] = "/dev/ttyACM0";

//file pointer for log
FILE *fp ;
char file_name[] = "/root/802.11p/log_Tracking_TX.txt";
unsigned int sleep_time = 10000;

//---------------------------------------------------------//
//                FUNCTIONS DECLARATIONS
//--------------------------------------------------------//
void reverse (char s[]);
void itoa(int n, char s[]);

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

//---------------------------------------------------------//
//                       MAIN
//--------------------------------------------------------//

int main() {

  char buf[BUFLEN];
  struct timespec timestamp;            //timestamp to add to payload
  double microsec = 0.0;                //varible for paste current timestamp in microseconds
  //---------------------------------------------------------//
  //                   SERIAL PART
  //--------------------------------------------------------//

  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open(port_name,  O_RDWR );
  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;


  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
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

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, UART_SPEED);
  cfsetospeed(&tty, UART_SPEED);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Allocate memory for serial
  char read_buf [300];
  char temp_string[20];
  char UTC_raw[20];
  char connection[2];
  char latitude_raw[20];
  char longitude_raw[20];
  char speed_raw[10];
  char heading_raw[10];
  char latitude_dir[2];
  char longitude_dir[2];

  int num_seq = 0;
  char num_seq_str[10000];
  
  //payload
  char payload[300];
  int lenght = 0;
  char buff_micro[100];


  //open a log file and write a header
  fp=fopen(file_name,"a");
  fprintf(fp,"UTC,Latitude_raw,Latitude_dir,Longitude_raw,Longitude_dir,speed_raw,heading_raw,seq_num,UID,Timestamp\n");
  fclose(fp);

while(1){

  memset(&read_buf, '\0', sizeof(read_buf));        //clean the buffer of precedent values
  memset(&temp_string, '\0', sizeof(temp_string));  //clean the buffer of precedent values
  memset(&payload, '\0', sizeof(payload));
  read(serial_port, &read_buf, sizeof(read_buf));

  if (read_buf[3] == 'R'){
    //only fing GPRMC data from GPS USB
    int i=0, j, k;

    for(j=0; j<9;j++){
        k=0;
        for (; read_buf[i] != ','; i++){

          temp_string[k] = read_buf[i];
          k++;
        }
        temp_string[k] = '\0';

        if(j==0){
            //first is GPRMC
            i++;
        }
        if(j==1){
            i++;
            strcpy(UTC_raw, temp_string);
            //printf("UTC : %s\n", UTC_raw);
        }
        if(j==2){
            i++;
            strcpy(connection, temp_string);
            //printf("connection : %s\n", connection);
        }
        if(j==3){
            i++;
            strcpy(latitude_raw, temp_string);
            //printf("latitude_raw : %s\n", latitude_raw);
        }
        if(j==4){
            i++;
            strcpy(latitude_dir, temp_string);
            //printf("latitude_dir : %s\n", latitude_dir);
        }
        if(j==5){
            i++;
            strcpy(longitude_raw, temp_string);
            //printf("longitude_raw : %s\n", longitude_raw);
        }
        if(j==6){
            i++;
            strcpy(longitude_dir, temp_string);
            //printf("longitude_dir : %s\n", longitude_dir);
        }
        if(j==7){
            i++;
            strcpy(speed_raw, temp_string);
            //printf("speed_raw : %s\n", speed_raw);
        }
        if(j==8){
            strcpy(heading_raw, temp_string);
            //printf("heading : %s\n", heading_raw);
        }
    }

  }

  //collect current time
  clock_gettime(CLOCK_REALTIME, &timestamp); //prendo il timestamp da mettere nel payload
  //timestamp conversion in microseconds
  microsec = timestamp.tv_sec * 1000000LL + timestamp.tv_nsec / 1000;

  //---------------------------------------------------------//
  //           CALCULATIONS SECTION
  //--------------------------------------------------------//

  fp=fopen(file_name,"a");

  if(connection[0] == 'A'){

    //payload + dummy to reach 300 bytes
    strcat(payload, UTC_raw);
    strcat(payload, ",");
    strcat(payload, latitude_raw);
    strcat(payload, ",");
    strcat(payload, latitude_dir);
    strcat(payload, ",");
    strcat(payload, longitude_raw);
    strcat(payload, ",");
    strcat(payload, longitude_dir);
    strcat(payload, ",");
    strcat(payload, speed_raw);
    strcat(payload, ",");
    strcat(payload, heading_raw);
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

    lenght = strlen(payload);

    //write the log    
    fprintf(fp,"%s,%s,%s,%s,%s,%d,%s,%lf\n",UTC_raw,latitude_raw,latitude_dir,longitude_raw,longitude_dir,num_seq,UID,microsec);
    printf("Collected GPS TRACK -> seq_num of GPS data collected %d\n", num_seq);
    //clean buffer
    memset(buf,'\0', BUFLEN);
  }
  
  fclose(fp);

  //END WHILE   
}

  close(serial_port);
  return 0; // success
}
