#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>
#include <iostream>

int main(int argc, char *argv[]) {
    int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

    struct termios tty;
struct termios tty_old;
memset (&tty, 0, sizeof tty);

/* Error Handling */
if ( tcgetattr ( USB, &tty ) != 0 ) {
   std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
}

/* Save old tty parameters */
tty_old = tty;

/* Set Baud Rate */
cfsetospeed (&tty, (speed_t)B9600);
cfsetispeed (&tty, (speed_t)B9600);

/* Setting other Port Stuff */
tty.c_cflag     &=  ~PARENB;            // Make 8n1
tty.c_cflag     &=  ~CSTOPB;
tty.c_cflag     &=  ~CSIZE;
tty.c_cflag     |=  CS8;

tty.c_cflag     &=  ~CRTSCTS;           // no flow control
tty.c_cc[VMIN]   =  1;                  // read doesn't block
tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

/* Make raw */
cfmakeraw(&tty);

/* Flush Port, then applies attributes */
tcflush( USB, TCIFLUSH );
if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) {
   std::cout << "Error " << errno << " from tcsetattr" << std::endl;
}

int n = 0,
    spot = 0;
char buf = '\0';

/* Whole response*/
char response[1024];
memset(response, '\0', sizeof response);

do {
    USB = read( USB, &buf, 1 );
    sprintf( &response[spot], "%c", buf );
    spot += USB;
} while( buf != '\r' && n > 0);

if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
}
else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
}
else {
    std::cout << "Response: " << response << std::endl;
}

    return 0;
}
