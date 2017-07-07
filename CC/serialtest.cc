#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <msgpack.hpp>


int openPort(char const *port);
char readPort(int fd);

int openPort(char const *port) {
    int fd;
    fd = open(port, O_RDWR |  O_NDELAY, O_SYNC);
    //printf("Port:%s\n",port);
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
    std::cout << fd << std::endl;
    return fd;
}

char readPort(int fd) {
    char buf[1024];
    fd = read(fd, buf, 1);
    if (fd<0) {
        return -1;
    }
    return *buf;
}


int main(int argc, char *argv[]) {
  int fd=0;
  fd=openPort("/dev/ttyACM0");
  std::cout << "Here" << std::endl;
  char buffer;
  while (1) {
      buffer=readPort(fd);
      std::cout << buffer << std::endl;
  }
  return 0;
}
