#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>

#define sensorRead 1
#define sensorReadByte '1'
#define servoRead 2
#define servoReadByte '2'
#define servoWrite 3
#define servoWriteByte '3'
#define acknowledgeSize 'a'

int openPort(char const *port);
std::string readPort(int fd);
int teensyRequest(int fd, int op);
int msgSize(int fd);

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
    char byte='0';

    switch (op) {
        case sensorRead:byte=sensorReadByte;
            break;
            case servoRead:byte=servoReadByte;
                break;
                case servoWrite:byte=servoWriteByte;
                    break;
    }

    fd = write(fd, &byte, sizeof byte);
    std::cout << byte << std::endl;
    return 0;
}

int getMsgSize(int fd) {
    char buf[6]{};
    std::string temp;
    int msgSize;
    char ack=acknowledgeSize;

    do {
        read(fd , buf, sizeof buf);
    } while(buf[5]!='}');

        for (size_t i = 0; i < sizeof buf; i++) {
            temp.push_back(buf[i]);
        }

    std::cout << temp << std::endl;
    msgSize=std::stoi(temp);
    write(fd, &ack, sizeof ack);
    return msgSize;
}

std::string readPort(int fd) {
    char buf[6]{};
    std::string temp;

    do {
        read(fd, buf, sizeof buf);
    } while(buf[5]!='}');
    std::cout << buf << std::endl;

        for (size_t i = 0; i < sizeof buf; i++) {
            temp.push_back(buf[i]);
        }

    return temp;
}

int main(int argc, char *argv[]) {
  int fd{0};
  std::string temp;
  fd=openPort("/dev/ttyACM0");
  teensyRequest(fd, sensorRead);
  getMsgSize(fd);
  //temp=readPort(fd);
  std::cout << temp << std::endl;
  return 0;
}
