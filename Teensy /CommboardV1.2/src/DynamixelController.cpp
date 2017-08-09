//#include <DynamixelMessage.h>
#include <Arduino.h>
#include <string>
#include <vector>
#include <cstdint>
#include <ArduinoJson.h>
#include "DynamixelController.hpp"


void DynamixelController::scanPort() {
    scanMode = true;

    for (int i=0;i<254;i++){
        idMap[i]=0;
    }
    uint8_t search[16];
    for(int i=0;i<16;i++)
    {
        search[i]=0;
    }

    for(int k=0;k<8;k++){
        for (int j=32*k;j<32*(k+1);j++){
            search[0]=255;
            search[1]=255;
            search[2]=j;
            search[3]=4;
            search[4]=2;
            search[5]=3;
            search[6]=1;
            DynamixelMessage* ScanMessage1= new DynamixelMessage(search); //messages are being generated and put
            DynamixelMessage* ScanMessage2= new DynamixelMessage(search); //
            DynamixelMessage* ScanMessage3= new DynamixelMessage(search); //
            queue1.push(ScanMessage1);                                                   //messages are being pushed
            queue2.push(ScanMessage2);                                                   //into their corresponding queues
            queue3.push(ScanMessage3);
        }

        send1();
        send2();
        send3();
        while(busy1 && busy2 && busy3){
            delay(10);                                                                  //Waiting for ports to finish writing all messages
        }

        scanMode=false;
    }
}

void DynamixelController::UartInit() {

    event1.txEventHandler = txEvent1;             //defines the function to trigger for sent bytes on Port 1 of the Teensy
    event1.rxEventHandler = rxEvent1;             //defines the function to trigger for received bytes on Port 1 of the Teensy
    event1.rxBufferSizeTrigger = 1;               //defines how many bytes have to enter the input buffer until the interrupt triggers the interrupt function
    event1.begin(1000000);                        //defines the baudrate for the corresponding port
    event1.clear();                               //clears

    event2.txEventHandler = txEvent2;
    event2.rxEventHandler = rxEvent2;
    event2.rxBufferSizeTrigger = 1;
    event2.begin(1000000);
    event2.clear();

    event3.txEventHandler = txEvent3;
    event3.rxEventHandler = rxEvent3;
    event3.rxBufferSizeTrigger=1;
    event3.begin(1000000);
    event3.clear();
}

void DynamixelController::readFromUSB() {
    static byte idx = 0;
    static boolean inProgress = false;
    static boolean newData = false;
    char  c;
    char buf[bufLen];
    String s;

    while (Serial.available() > 0 && newData == false) {
        c = Serial.read();
        if (inProgress == true) {
            if (c != '}') {
                buf[idx] = c;
                idx++;
                if (idx >= bufLen) {
                    idx = bufLen-1;
                }
            }
            else {
                buf[idx] = '\0';
                idx = 0;
                newData = true;
                inProgress = false;
                s = buf;
                jsonParser(s);
            }
        }
        else if (c == '{') {
            inProgress = true;
            buf[idx] = '{';
            idx++;
        }

    }
    //delay(300);
    newData = false;
    s = buf;
    s.append('}');
    //Serial.println(s);
    parseJsonString(s);
}

void DynamixelController::parseJsonString(String s) {
    Vector<int> velVec;
    StaticJsonBuffer<jsonBufLen> jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(s);
    velVec.push_back(root["velLeft"]);
    velVec.push_back(root["velRight"]);

    servoWritePcktConstructor(&velVec);
}

void servoWritePcktConstructor(Vector<int>* velArray) {
    uint8_t servoPckt[8]{0};
    size_t checkSum = 0, checkSumPos = 0, len = 0;
    int val = 0;
    for (size_t id = 1; id <= 2; id++) {
        servoPckt[0] = FF;
        servoPckt[1] = FF;
        servoPckt[2] = id;
        servoPckt[3] = _LENGHT;
        servoPckt[4] = _WRITE_SERVO_DATA;
        val = velArray->at(id);
        if (val < 0) {
            val *= (-1);
            val += 1024;
        }
        servoPckt[5] = val & 255;
        servoPckt[6] = val >> 8;
        len = servoPckt[lenPos] + lenConst;
        //checkSumPos =  len + 1;
            for (size_t i = 2; i < len; i++) {
                checkSum += servoPckt[i];
            }
        checkSum = ~(checkSum) & 255;
        servoPckt[7] = checkSum;
        velArray->clear();

        writeToUART(&servoPckt[0]);
    }
}


void DynamixelController::writeToUART() {
    void writeToUART(uint8_t* servoPckt) {
        DynamixelMessage* USBMessage = new DynamixelMessage(servoPckt);

        if (idMap[servoPckt[2]] == 1 || scanMode) {
            pushToQueu1(USBMessage);
        }
        else if (idMap[servoPckt[2]] == 2 || scanMode) {
            pushToQueue2(USBMessage);
        }
        else if (idMap[servoPckt[2]] == 3 || scanMode) {
            pushToQueue3(USBMessage);
        }
        else if (idMap[servoPckt[2]] == 0) {
            delete USBMessage;
        }
    }
}

void DynamixelController::txEvent( IntervalTimer* timer,                                               //Timer interrupt that triggers a function if a message does not
              void (*noMessageReceivalFunctionPointer)(void)){                    //reply after the time specified in begin(). The timer uses Âµseconds as a unit.
  timer->priority(255);
  timer->begin(noMessageReceivalFunctionPointer,300);
}

void DynamixelController::txEvent1(){
  txEvent(&txTimer1, noMessageReceival1);
}

void DynamixelController::txEvent2(){
  txEvent(&txTimer2, noMessageReceival2);
}

void DynamixelController::txEvent3(){
  txEvent(&txTimer3, noMessageReceival3);
}
