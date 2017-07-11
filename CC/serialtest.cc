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
    char msgSize[3]{};
    std::string temp;
    int size;
        do {
            fd = read(fd , msgSize, sizeof msgSize);
        } while(msgSize[3]!=' ');
    for (size_t i = 0; i < sizeof msgSize; i++) {
        temp.push_back(msgSize[i]);
    }
    size=std::stoi(temp);
    return size;
}

std::string readPort(int fd, int bufferSize) {
    char buf[bufferSize]{};
    std::string temp;
        do {
            fd = read(fd, buf, sizeof buf);
        } while(buf[bufferSize]!='}');
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
  temp=readPort(fd, getMsgSize(fd));
  std::cout << temp << std::endl;
  return 0;
}
