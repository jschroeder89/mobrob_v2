#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <ArduinoJson.h>

//Defines
#define sensorRead 1
#define sensorReadByte '1'
#define servoRead 2
#define servoReadByte '2'
#define servoWrite 3
#define servoWriteByte '3'
#define bufLen 512

//Prototypes
int openPort(char const *port);
std::string readPort(int fd);
int teensyRequest(int fd, int op);
int jsonParser(std::string s);

int openPort(char const *port) {
    int fd;
    fd = open(port, O_RDWR | O_NDELAY);

        if  (fd == -1) {
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
    char byte;

    switch (op) {
        case sensorRead:
        byte = sensorReadByte;
        break;
            case servoRead:
            byte = servoReadByte;
            break;
                case servoWrite:
                byte = servoWriteByte;
                break;
    }

    write(fd, &byte, sizeof byte);
    return 0;
}

std::string readPort(int fd) {

    std::string s;
    char buf[bufLen];
    int n = 0, nbytes = 0;

        do {
            n = read(fd, buf+nbytes, bufLen-nbytes);

            if ((n == -1) && (errno == EINTR)) {
                continue;
            }
            if ((n == 0) && (nbytes == 0)) {
                continue;
            }
            if (nbytes == 0) {
            }
            if (n == -1) {
                return "";
            }
            nbytes += n;
            if (buf[nbytes-1] == '}') {
                buf[nbytes] = '\0';
                s = buf;
                break;
            }
        } while(nbytes <= bufLen);
    return s;
}

int jsonParser(std::string json) {

    StaticJsonBuffer<500> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(json);
    std::string dataType = root["data"];

    if (dataType == "sensor") {
        int dataF = root["F"][1];
        int dataR = root["R"][0];
        int dataL = root["L"][0];
        int dataB = root["B"][0];

        std::cout << dataR << std::endl;
    }


    return 0;
}

void createJsonString(int fd) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char buffer[256];
    root["data"] = "servoVels";
    //root["velLeft"] = 20;
    //root["velRight"] = 30;

    JsonArray& servoData = root.createNestedArray("servoData");
    servoData.add(100);
    servoData.add(120);
    root.printTo(buffer, sizeof buffer);
    write(fd, &buffer, sizeof buffer);
    std::cout << root << std::endl;
}


int main(int argc, char *argv[]) {
  int fd = 0;
  std::string s;
  fd = openPort("/dev/ttyACM0");
  createJsonString(fd);
  //teensyRequest(fd, sensorRead);
  //jsonParser(readPort(fd));
  return 0;
}
