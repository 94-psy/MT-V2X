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

//define ip brodcast for soket
//#define SERVER "192.168.1.5"
#define IPBRD "192.168.50.255"
#define BUFLEN 300	//Max length of buffer
#define PORT 8888	//The port on which to send data

//---------------------------------------------------------//
//                GLOBAL VARIBLES
//--------------------------------------------------------//

//curret id of mikrotik
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

//---------------------------------------------------------//
//                FUNCTIONS DECLARATIONS
//--------------------------------------------------------//
void reverse (char s[]);
void itoa(int n, char s[]);
double F_velocity (double v1, double v2);
double F_timeout (double UTC1, double UTC2);
double F_distance (double lat1, double lat2, double long1, double long2, char *lat1_d, char *lat2_d, char *long1_d, char *long2_d);
double F_heading (double h1, double h2);
int F_compare (double heading, double speed, double distance, double timeout);
void die(char *s);

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

double F_velocity (double v1, double v2){

    double final_velocity = 0.0;

    final_velocity=fabs(v1-v2);                         //differece in module
    final_velocity = final_velocity/1.9438444924406;    //convert knot to m/s

    return final_velocity;
}

double F_timeout (double UTC1, double UTC2){
    
    double final_UTC = 0.0;

    final_UTC=fabs(UTC1-UTC2);          //differece in module
    final_UTC = final_UTC/1000;         //convert microseconds to milliseconds

    return final_UTC;

}

double F_distance (double lat1, double lat2, double long1, double long2, char *lat1_d, char *lat2_d, char *long1_d, char *long2_d){

    int result = -1;
    double R, a, c, d, lat_a, lat_b, long_a, long_b;
    R = 6371000.0; // Radius of the earth (in meters)

    double dd1, n_lat1, n_lat2, n_long1, n_long2;
    dd1 = (((int)(lat1/100))*100);  //4437 --> 4400
    n_lat1 = dd1/100 + ((lat1 - dd1)/60);   

    dd1 = (((int)(lat2/100))*100);  
    n_lat2 = dd1/100 + ((lat2 - dd1)/60);   

    dd1 = (((int)(long1/100))*100);
    n_long1 = dd1/100 + ((long1 - dd1)/60);

    dd1 = (((int)(long2/100))*100);
    n_long2 = dd1/100 + ((long2 - dd1)/60);

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
    a = pow((sin(lat_b - lat_a) / 2.0), 2) +
        (cos(lat_a) * cos(lat_b) * pow(sin(long_b - long_a) / 2.0, 2));
    c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
    d = R * c;

    return d;
}

double F_heading (double h1, double h2){

    double diff_heading = 0.0;

    diff_heading=fabs(h1-h2); 
        if (diff_heading >= 180) 
            diff_heading=360-diff_heading;

    return diff_heading;
}

int F_compare (double heading, double speed, double distance, double timeout){

    int diff = 0;

    if(heading >= 4){       //heading
        trig_H = 1;
    }
    if(speed >= 0.5){       //speed
        trig_S = 1;
    }
    if(distance >= 4){      //distance
        trig_D = 1;
    }
    if(timeout >= 900){     //time out = 900 milliseconds
        trig_TO = 1;
    }
    if(trig_H == 1 || trig_D == 1 || trig_S == 1 || trig_TO == 1){
        diff = 1;
    }
    return diff;
}

void die(char *s)
{
	perror(s);
	exit(1);
}

//---------------------------------------------------------//
//                       MAIN
//--------------------------------------------------------//

int main() {

  struct sockaddr_in broadcastAddr;     //soket brodcast
  int s, l;
  unsigned int slen=sizeof(broadcastAddr);
  char buf[BUFLEN];
  struct timespec timestamp;            //timestamp to add to payload
  double microsec = 0.0;                //varible for paste current timestamp in microseconds


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

  //flag for first paket or not
  int first_packet = 0;
  int num_seq = 0;
  char num_seq_str[10000];
  
  //payload
  char payload[300];
  int lenght = 0;
  char buff_micro[100];

  //varibles for calculate trigger
  double old_heading = 0.0;
  double old_latitude = 0.0;
  double old_longitude = 0.0;
  double old_speed = 0.0;
  double old_UTC = 0.0;
  char old_latitude_dir[2];
  char old_longitude_dir[2]; 

  //open a log file and write a header
  fp=fopen(file_name,"a");
  fprintf(fp,"UTC,Latitude_raw,Latitude_dir,Longitude_raw,Longitude_dir,speed_raw,heading_raw,seq_num,UID,trig_S,trig_H,trig_D,trig_TO,Timestamp(ns)\n");
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

  //convertions
  double heading = atof(heading_raw);
  double latitude = atof(latitude_raw);
  double longitude = atof(longitude_raw);
  double speed = atof(speed_raw);
  double UTC = microsec;
  trig_H = 0;    
  trig_TO = 0;  
  trig_D = 0;     
  trig_S = 0;

  fp=fopen(file_name,"a");

  if(first_packet == 0 && connection[0] == 'A'){
    //for first packet and ready GPS data
    old_heading = heading;
    old_latitude = latitude;
    old_speed = speed;
    old_longitude = longitude;
    old_UTC = UTC;
    strcpy(old_longitude_dir, longitude_dir);
    strcpy(old_latitude_dir, latitude_dir);

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
    
    //send the message
	if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
	    die("sendto()");
	}

    //write the log        
    fprintf(fp,"%s,%lf,%s,%lf,%s,%lf,%lf,%d,%s,%d,%d,%d,%d,%lf\n",UTC_raw,latitude,latitude_dir,longitude,longitude_dir,speed,heading,num_seq,UID,trig_S,trig_H,trig_D,trig_TO,microsec);
    printf("Pyaload Sendt (%d Bytes) -> seq_num %d, TH: %d, TS: %d, TD: %d, TTO: %d\n", lenght, num_seq, trig_H, trig_S, trig_D, trig_TO);
    
    //for test carlo distance
    usleep(50000);
    //send the message
	if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
	    die("sendto()");
	}
    fprintf(fp,"%s,%lf,%s,%lf,%s,%lf,%lf,%d,%s,%d,%d,%d,%d,%lf\n",UTC_raw,latitude,latitude_dir,longitude,longitude_dir,speed,heading,num_seq,UID,trig_S,trig_H,trig_D,trig_TO,microsec);
    printf("Pyaload Sendt (%d Bytes) -> seq_num %d, TH: %d, TS: %d, TD: %d, TTO: %d\n", lenght, num_seq, trig_H, trig_S, trig_D, trig_TO);
    
    //clean buffer
    memset(buf,'\0', BUFLEN);
  }

  if(first_packet > 1){
    //next packets
    double diff_heading = 0.0;
    double diff_velocity = 0.0;
    double diff_distance = 0.0;
    double diff_timeout = 0.0;
    int send = -1;
    //calculate trigger for CAM messages
    diff_heading = F_heading(heading, old_heading);
    diff_velocity = F_velocity(speed, old_speed);
    diff_distance = F_distance(old_latitude, latitude, old_longitude, longitude, old_latitude_dir, latitude_dir, old_longitude_dir, longitude_dir);
    diff_timeout = F_timeout(UTC, old_UTC);
    //send = F_compare(diff_heading, diff_velocity, diff_distance, diff_timeout);
    send = 1;

    if (send == 1){

        //payload creation
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

        //send the message
	    if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
		    die("sendto()");
	    }
		
	    //clear the buffer by filling null, it might have previously received data
	    memset(buf,'\0', BUFLEN);

        //write log
        fprintf(fp,"%s,%lf,%s,%lf,%s,%lf,%lf,%d,%s,%d,%d,%d,%d,%lf\n",UTC_raw,latitude,latitude_dir,longitude,longitude_dir,speed,heading,num_seq,UID,trig_S,trig_H,trig_D,trig_TO,microsec);

        printf("Pyaload Sendt (%d Bytes) -> seq_num %d, TH: %d, TS: %d, TD: %d, TTO: %d\n", lenght, num_seq, trig_H, trig_S, trig_D, trig_TO);
        
        //for test carlo distance
        usleep(50000);
        //send the message
	    if (sendto(s, payload, strlen(payload) , 0 , (struct sockaddr *) &broadcastAddr, slen)==-1){
	        die("sendto()");
	    }
        fprintf(fp,"%s,%lf,%s,%lf,%s,%lf,%lf,%d,%s,%d,%d,%d,%d,%lf\n",UTC_raw,latitude,latitude_dir,longitude,longitude_dir,speed,heading,num_seq,UID,trig_S,trig_H,trig_D,trig_TO,microsec);
        printf("Pyaload Sendt (%d Bytes) -> seq_num %d, TH: %d, TS: %d, TD: %d, TTO: %d\n", lenght, num_seq, trig_H, trig_S, trig_D, trig_TO);
    
        old_heading = heading;
        old_latitude = latitude;
        old_speed = speed;
        old_longitude = longitude;
        old_UTC = UTC;
        strcpy(old_longitude_dir, longitude_dir);
        strcpy(old_latitude_dir, latitude_dir);
    }
  }
  
  fclose(fp);
  
  first_packet = 2;

  //END WHILE   
}

  close(serial_port);
  close(s);
  return 0; // success
}
