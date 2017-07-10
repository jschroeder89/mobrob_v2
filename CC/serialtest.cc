#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>

#define sensorRead 1
#define servoRead 2
#define servoWrite 3

int openPort(char const *port);
std::string readPort(int fd);
int teensyRequest(int fd, int op);


int openPort(char const *port) {
    int fd;
    fd = open(port, O_RDWR |  O_NDELAY);
        if  (fd==-1) {
            perror("Port not found");
            return -1;
        }
        /*struct termios options;

        tcgetattr(fd, &options);
        cfsetispeed(&options, B9600);
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_iflag &= ~(BRKINT|ICRNL|ISTRIP|IXON);
        options.c_oflag = 0;
        options.c_lflag &= ~(ECHO|ICANON|IEXTEN|ISIG);
        tcflush( fd, TCIFLUSH );
        tcsetattr(fd, TCSANOW, &options);*/
    return fd;
}

int teensyRequest(int fd, int op) {
    char byte='0';
    switch (op) {
        case sensorRead:byte='1';
        break;
        case servoRead:byte='2';
        break;
        case servoWrite:byte='3';
        break;
    }
    fd = write(fd, &byte, sizeof byte);
    std::cout << byte << std::endl;
    return 0;
}

std::string readPort(int fd) {
    char buf[6];
    std::string temp;
    do {
        fd = read(fd, buf, sizeof buf);
    } while(buf[5]!='}');
        for (size_t i = 0; i < sizeof buf; i++) {
            temp.push_back(buf[i]);
        }
    return temp;
}

int main(int argc, char *argv[]) {
  int fd{0};
  std::string temp;
  fd=openPort("/dev/ttyACM0");
  //temp=readPort(fd);
  //std::cout << temp << std::endl;
  teensyRequest(fd, servoRead);
  return 0;
}
