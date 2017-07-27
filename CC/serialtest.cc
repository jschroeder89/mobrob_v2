#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <ArduinoJson.h>

//Defines
#define sensorRead 1
#define sensorReadByte '1'
#define servoRead 2
#define servoReadByte '2'
#define servoWrite 3
#define servoWriteByte '3'
#define bufLen 512
#define jsonBufLen 512

//Prototypes
int openPort(char const *port);
std::string readPort(int fd);
int teensyRequest(int fd, int op);
std::vector<int> jsonParser(std::string s);
void setVelocity(int fd, int velLeft, int velRight);


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

std::vector<std::vector<int> >jsonSensorParser(std::string json) {
    StaticJsonBuffer<jsonBufLen> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(json);
    std::vector<int> FL, L, R, B;
    std::vector<std::vector<int> > sensorData;

    for (size_t i = 0; i < 16; i++) {
        FL.push_back(root["FL"][i]);
    }
    for (size_t i = 0; i < 8; i++) {
        L.push_back(root["L"][i]);
        B.push_back(root["B"][i]);
        R.push_back(root["R"][i]);
    }
    sensorData.push_back(FL);
    sensorData.push_back(L);
    sensorData.push_back(B);
    sensorData.push_back(R);

    return sensorData;
}

std::vector<int> jsonServoParser(std::string json) {
    StaticJsonBuffer<jsonBufLen> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(json);

    std::vector<int> servoData;
    servoData.push_back(root["velLeft"]);
    servoData.push_back(root["velRight"]);

    return servoData;
}

void setVelocity(int fd, int velLeft, int velRight) {
    StaticJsonBuffer<bufLen> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    char buffer[bufLen];

    root["data"] = "servoVels";
    root["velLeft"] = velLeft;
    root["velRight"] = velRight;

    root.printTo(buffer, sizeof buffer);
    int n = write(fd, &buffer, sizeof buffer);
    if (n > 0) {
        std::cout << buffer << std::endl;
    }
}

int main(int argc, char *argv[]) {
  int fd = 0;
  std::string s;

  fd = openPort("/dev/ttyACM0");
  teensyRequest(fd, servoWrite);
  setVelocity(fd, 50, 50);
  //jsonParser(readPort(fd));
  return 0;
}
