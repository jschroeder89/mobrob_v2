#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>

#define sensorRead 1
#define sensorReadByte '1'
#define servoRead 2
#define servoReadByte '2'
#define servoWrite 3
#define servoWriteByte '3'

int openPort(char const *port);
std::string readPort(int fd);
int teensyRequest(int fd, int op);

int openPort(char const *port) {
    int fd;
    fd = open(port, O_RDWR | O_NDELAY);

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
    char byte{};

    switch (op) {
        case sensorRead:byte=sensorReadByte;
            break;
            case servoRead:byte=servoReadByte;
                break;
                case servoWrite:byte=servoWriteByte;
                    break;
    }

    write(fd, &byte, sizeof byte);
    return 0;
}

std::string readPort(int fd) {
    std::string temp;
    char c;

        do {
            read(fd, &c, 1);
            temp.push_back(c);
        } while(c!='}');
        std::cout << temp << std::endl;
    return temp;
}

int main(int argc, char *argv[]) {
  int fd{0};
  std::string temp;
  fd=openPort("/dev/ttyACM0");
  teensyRequest(fd, sensorRead);
  temp=readPort(fd);
  return 0;
}
