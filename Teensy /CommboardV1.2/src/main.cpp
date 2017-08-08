#include <UartEvent.h>
#include <DynamixelMessage.h>
#include <QueueArray.h>
#include <Arduino.h>
#include <i2c_t3.h>
#include <string>
#include <vector>
#include <cstdint>
#include <ArduinoJson.h>

//AD-Channels-Addresses
#define AD0 0x08 // 0001000
#define AD1 0x09 // 0001001
#define AD2 0x1B // 0001010
#define AD3 0x0B // 0001011
#define AD4 0x18 // 0011000
#define AD5 0x19 // 0011001
#define AD6 0x1A // 0011010
#define AD7 0x0A // 0011011
#define AD8 0x28 // 0101000

//AD-Converter Channels
#define CH0       0x88 // 10001000
#define CH1       0xC8 // 11001000
#define CH2       0x98 // 10011000
#define CH3       0xD8 // 11011000
#define CH4       0xA8 // 10101000
#define CH5       0xE8 // 11101000
#define CH6       0xB8 // 10111000
#define CH7       0xF8 // 11111000
#define CHGLOBAL  0xD6 // 11010110

#define sensorReadByte '1'
#define servoReadByte '2'
#define servoWriteByte '3'

#define FF 255
#define _LENGHT 5
#define _NUM_OF_BYTES_TO_READ 2
#define NumOfSensors 5
#define lenPos 3
#define lenConst 2

#define sensorRead 1
#define servoRead 2
#define servoWrite 3
#define bufLen 512
#define jsonBufLen 256
//Arrays
uint8_t address[] = {AD0, AD1, AD2, AD3, AD4};
uint8_t channel[] = {CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7};

//boolean newData = false;

Uart1Event event1;//initialize UART A of the Teensy for enhanced features like DMA capability
Uart2Event event2;//initialize UART B ""
Uart3Event event3;//initialize UART C ""

IntervalTimer txTimer1;
IntervalTimer txTimer2;
IntervalTimer txTimer3;

QueueArray <DynamixelMessage*> queue1(50);              //queues for the individual ports to hold the messages
QueueArray <DynamixelMessage*> queue2(50);              //
QueueArray <DynamixelMessage*> queue3(50);              //

#define MAX_LENGTH_OF_MESSAGE 259
volatile uint8_t rcvdPkt1[MAX_LENGTH_OF_MESSAGE];
volatile uint8_t rcvdPkt2[MAX_LENGTH_OF_MESSAGE];
volatile uint8_t rcvdPkt3[MAX_LENGTH_OF_MESSAGE];

volatile uint16_t posInArray1 = 0;
volatile uint16_t posInArray2 = 0;
volatile uint16_t posInArray3 = 0;

volatile uint16_t numOfBytesToRead1 = 4;
volatile uint16_t numOfBytesToRead2 = 4;
volatile uint16_t numOfBytesToRead3 = 4;

Vector<uint8_t> messageVector1;           //Vectors to hold the messages before putting them into the queues.
Vector<uint8_t> messageVector2;           //Should behave like the regular C++ standard-class vector-type.
Vector<uint8_t> messageVector3;

volatile bool sync1 = true;
volatile bool sync2 = true;
volatile bool sync3 = true;

volatile bool busy1=false;
volatile bool busy2=false;
volatile bool busy3=false;

volatile uint8_t resendCounter1=0;
volatile uint8_t resendCounter2=0;
volatile uint8_t resendCounter3=0;

volatile bool usbMode=false;

                                        //will be memorized in the idMap. scanMode is being set
                                        //in the scanPort function automatically
uint8_t rcvdPktUsb[MAX_LENGTH_OF_MESSAGE];
uint8_t posInArrayUsb=0;


void rxEvent1();              //interrupt functions for the input buffer on the UART ports
void rxEvent2();
void rxEvent3();

void txEvent1();                            //interrupt functions for the output buffer on the UART ports
void txEvent2();
void txEvent3();

void rxResync1();
void rxResync2();
void rxResync3();

void noMessageReceival1();                //timer-interrupt functions for timed out messages
void noMessageReceival2();
void noMessageReceival3();

void rxSerialEventUsb();                   //function that listens for bytes on the Serial (USB) port.

void send1();
void send2();
void send3();

void pushToQueue1(DynamixelMessage* messageToPush);
void pushToQueue2(DynamixelMessage* messageToPush);
void pushToQueue3(DynamixelMessage* messageToPush);
void scanPort();

void readSensorData();
void writeToUSB(JsonObject& root);
void convertSensorDataToJson();
void convertServoDataToJson(int* dataArray);
void parseJsonString(String s);
void servoWritePcktConstructor(uint8_t* servoPckt);
void servoReadPcktConstructor(int* servoPckt);
void writeToUART();
void convertVelocities(Vector<int>* servoPckt);
void readStatusPckt(uint8_t* rcvdPkt);
void readFromUSB();

class scanIDs {
    public:
        volatile uint8_t idMap[256];
        void scanPort();
        volatile bool scanMode=false;
};

void scanIDs::scanPort() {
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

void setup(){
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
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

  digitalWrite(13, HIGH);
  delay(4000);
  scanIDs portInit;
  portInit.scanPort();
  //scanPort();
  delay(2000);
  digitalWrite(13, LOW);
}


int getI2CSensorData(int add, int ch)
{
  Wire.beginTransmission(address[add]);     // slave addr
  Wire.write(channel[ch]);
  int status = Wire.endTransmission();
  if (status == 0)
  {
    Wire.requestFrom(address[add], 2, I2C_STOP);
    while (Wire.available()) {
      int byte1 = Wire.readByte();
      int byte2 = Wire.readByte();
      int number = byte2 | byte1 << 8;

      return number;
    }
  }
  return -1;
}

void readFromUSB() {
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
                /*s = buf;
                jsonParser(s);*/
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

void writeToUART(uint8_t* servoPckt) {
    scanIDs portInit;
    DynamixelMessage* USBMessage = new DynamixelMessage(servoPckt);

    if (portInit.idMap[servoPckt[2]] == 1 || portInit.scanMode) {
        pushToQueue1(USBMessage);
    }
    else if (portInit.idMap[servoPckt[2]] == 2 || portInit.scanMode) {
        pushToQueue2(USBMessage);
    }
    else if (portInit.idMap[servoPckt[2]] == 3 || portInit.scanMode) {
        pushToQueue3(USBMessage);
    }
    else if (portInit.idMap[servoPckt[2]] == 0) {
        delete USBMessage;
    }
}

void readStatusPckt(Vector<int>* statusPckt) {
    if (statusPckt->at(0) != 0) {
        convertVelocities(statusPckt);
    }
}

void convertVelocities(Vector<int>* servoPckt) {
    int lowByte = 0, highByte = 0;
    static int velLeft, velRight;
    static boolean velLeftNotEmpty;
    static boolean velRightNotEmpty;

    if (servoPckt->at(0) == 1 && velLeftNotEmpty == true) {
        lowByte = lowByte & 255;
        highByte = highByte << 8;
        velLeft = lowByte + highByte;
        velLeftNotEmpty = false;
        if (velLeft >= 1024) {
            velLeft -= 1024;
            velLeft *= (-1);
        }
    }
    else if (servoPckt->at(0) == 2 && velRightNotEmpty == true) {
        lowByte = lowByte & 255;
        highByte = highByte << 8;
        velRight = lowByte +  highByte;
        velRightNotEmpty = false;
        if (velRight > 1024) {
            velRight -= 1024;
        }
        else {
            velRight *= (-1);
        }
    }
    if (velLeftNotEmpty && velRightNotEmpty == false) {
        int velArray[2] = {0};
        velArray[0] = velLeft;
        velArray[1] = velRight;
        velLeft = 0, velRight = 0;
        velLeftNotEmpty = true;
        velRightNotEmpty = true;

        convertServoDataToJson(&velArray[0]);
    }
}

void servoReadPcktConstructor() {
    uint8_t servoPckt[8]{0};
    size_t checkSum = 0, checkSumPos = 0, len = 0;
    for (size_t id = 1; id <=2; id++) {
        servoPckt[0] = FF;
        servoPckt[1] = FF;
        servoPckt[2] = id;
        servoPckt[3] = _LENGHT;
        servoPckt[4] = _READ_SERVO_DATA;
        servoPckt[5] = SERVO_REGISTER_PRESENT_SPEED;
        servoPckt[6] = _NUM_OF_BYTES_TO_READ;
        len = servoPckt[lenPos] + lenConst;
        //checkSumPos =  len + 1;
            for (size_t i = 2; i < len; i++) {
                checkSum += servoPckt[i];
            }
        checkSum = ~(checkSum) & 255;
        servoPckt[7] = checkSum;

        writeToUART(&servoPckt[0]);
    }
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

void parseJsonString(String s) {
    Vector<int> velVec;
    StaticJsonBuffer<jsonBufLen> jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(s);
    velVec.push_back(root["velLeft"]);
    velVec.push_back(root["velRight"]);

    servoWritePcktConstructor(&velVec);
}

void convertServoDataToJson(int* dataArray) {
    StaticJsonBuffer<jsonBufLen> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();

    root["data"] = "servo";
    root["velLeft"] = dataArray[0];
    root["velRight"] = dataArray[1];
    writeToUSB(root);
}

void writeToUSB(JsonObject& root) {
    root.printTo(Serial);
}

void convertSensorDataToJson(int sensorData[][8]) {
    StaticJsonBuffer<bufLen> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["data"] = "sensor";
    JsonArray& sensorFront = root.createNestedArray("F");
    JsonArray& sensorRight = root.createNestedArray("R");
    JsonArray& sensorLeft = root.createNestedArray("L");
    JsonArray& sensorBack = root.createNestedArray("B");
    for (size_t i = 0; i <= 2; i++) {
        for (int j = 0; j < 8; j++) {
            sensorFront.add(sensorData[i][j]);
            sensorLeft.add(sensorData[4][j]);
            sensorBack.add(sensorData[3][j]);
            sensorRight.add(sensorData[2][j]);
        }
    }
    writeToUSB(root);
}

void readSensorData() {
    int sensorData[5][8] = {0};
    for (size_t i = 0; i < 2; i++) {
        for (size_t j = 0; j < 8; j++) {
            sensorData[i][j] = getI2CSensorData(i, j);
            //front.push_back(getI2CSensorData(i, j));
        }
    }
    for (size_t i = 0; i < 8; i++) {
        sensorData[4][i] = getI2CSensorData(4, i);
        sensorData[3][i] = getI2CSensorData(3, i);
        sensorData[2][i] = getI2CSensorData(2, i);
    }
    convertSensorDataToJson(sensorData);
}

void loop() {
    if (Serial.available() > 0) {
        char requestByte = Serial.read();
        switch (requestByte) {
            case sensorReadByte:
                readSensorData();
                break;
            case servoReadByte:
                servoReadPcktConstructor();
                break;
            case servoWrite:
                readFromUSB();
                break;
        }
    }
}


/*void rxSerialEventUsb(){
 posInArrayUsb=0;
 if (!Serial.available()){
   return;
 }
 while(Serial.available()){

   if(posInArrayUsb==0){
     rcvdPktUsb[posInArrayUsb]=Serial.read();
     posInArrayUsb++;
   }
   if(posInArrayUsb>=3){
     if(rcvdPktUsb[0] == 255 && rcvdPktUsb[1] == 255){
       while(posInArrayUsb<rcvdPktUsb[3]+4){
         rcvdPktUsb[posInArrayUsb]=Serial.read();
         posInArrayUsb++;
       }
         if ((posInArrayUsb)==(rcvdPktUsb[3]+4)){
         uint8_t testchksum=0;
         for(int p=2;p<=rcvdPktUsb[3]+2;p++){                 //checksum test for message from USB
             testchksum=testchksum+rcvdPktUsb[p];
         }
         testchksum=~(testchksum)&255;
         if(testchksum!=rcvdPktUsb[posInArrayUsb-1]){
         }else if(testchksum==rcvdPktUsb[posInArrayUsb-1]){//Message seems to look good.Creation of Dynamixel Object and putting into Queue
           //Serial.println("Checksum correct, creating message object");
           DynamixelMessage* USBMessage=new DynamixelMessage(rcvdPktUsb);
           if(!USBMessage){
               return;
           }
           for(int i=0;i<=posInArrayUsb;i++)
           {
            // Serial.print(i);
            // Serial.print(": ");
            // Serial.println(rcvdPktUsb[i]);
           }

           if (idMap[rcvdPktUsb[2]] == 1 || scanMode){
             pushToQueue1(USBMessage);
           }else if(idMap[rcvdPktUsb[2]] == 2 || scanMode){
             pushToQueue2(USBMessage);
           }else if(idMap[rcvdPktUsb[2]] == 3 || scanMode){
             pushToQueue3(USBMessage);
           }else if(idMap[rcvdPktUsb[2]] == 0){
             //Serial.println("Could not find this Servo!");
             delete USBMessage;
           }
           posInArrayUsb=0;
           for(int i=0;i<MAX_LENGTH_OF_MESSAGE;i++)
           {
             rcvdPktUsb[i]=0;
           }
         }
       }
     }
   }else{
     if(posInArrayUsb<3){
       rcvdPktUsb[posInArrayUsb]=Serial.read();
       posInArrayUsb++;
       if(rcvdPktUsb[0] == 255 && rcvdPktUsb[1] == 255){//Found Packet prefix, trying to retreive ID+Length
         rcvdPktUsb[posInArrayUsb]=Serial.read();
         posInArrayUsb++;
         rcvdPktUsb[posInArrayUsb]=Serial.read();
         posInArrayUsb++;
       }
     }
   }
 }
}


void scanPort(){

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
}*/

void noMessageReceival( UartEvent*          event,
                        IntervalTimer*      timer,
                        volatile uint8_t*   resendCounter,
                        void(*sendFunctionPointer)(void),
                        Vector<uint8_t>*    messagevector){
    timer->end();
    if((*resendCounter)<2){
      event->write(messagevector->data(),messagevector->size());
      (*resendCounter)+=1;
    }else{
      (*resendCounter)=0;
      sendFunctionPointer();
    }
}



void noMessageReceival1(){
  noMessageReceival(&event1, &txTimer1, &resendCounter1,send1, &messageVector1);
}

void noMessageReceival2(){
    noMessageReceival(&event2, &txTimer2, &resendCounter2,send2, &messageVector2);
}

void noMessageReceival3(){
    noMessageReceival(&event3, &txTimer3, &resendCounter3,send3, &messageVector3);
}


void txEvent( IntervalTimer* timer,                                               //Timer interrupt that triggers a function if a message does not
              void (*noMessageReceivalFunctionPointer)(void)){                    //reply after the time specified in begin(). The timer uses Âµseconds as a unit.
  timer->priority(255);
  timer->begin(noMessageReceivalFunctionPointer,300);
}

void txEvent1(){
  txEvent(&txTimer1, noMessageReceival1);
}

void txEvent2(){
  txEvent(&txTimer2, noMessageReceival2);
}

void txEvent3(){
  txEvent(&txTimer3, noMessageReceival3);
}


void rxEvent( UartEvent*          event,
              IntervalTimer*      timer,
              volatile bool*      sync,
              volatile uint16_t*  posInArray,
              volatile uint16_t*  numOfBytesToRead,
              volatile uint8_t*   rcvdPkt,
              uint8_t             uartInt,
              volatile uint8_t*   resendCounter,
              void (*rxResyncFunctionPointer)(void),
              void(*sendFunctionPointer)(void)){
                  scanIDs portInit;
                  Vector<int> statusPckt;
  if ((*sync) == true){
    if ((*posInArray) >= 4){
      (*numOfBytesToRead) = rcvdPkt[3] +4;
    } else {
      (*numOfBytesToRead) = 4;
    }
    while (event->available() && (*posInArray) < (*numOfBytesToRead)){
      rcvdPkt[*posInArray] = event->read();
      (*posInArray)+=1;
    }

    if ((*posInArray) > 4 && (*posInArray) >= rcvdPkt[3] + 4){//received a complete packet from Dynamixel
      timer->end();
      *resendCounter=0;

      uint8_t testchksum=0;
      for(int p=2;p<=rcvdPkt[3]+2;p++){
          testchksum=testchksum+rcvdPkt[p];
      }
      testchksum=~(testchksum)&255;
      if(testchksum!=rcvdPkt[*posInArray-1]){
        //Serial.println("Wrong checksum from Servo");
        event->setRxEventHandler(rxResyncFunctionPointer);
        (*posInArray) = 0;
        event->setRxBufferSizeTrigger(1);
      }
      //Serial.println("Correct checksum from Servo");
      if(portInit.scanMode){
        portInit.idMap[rcvdPkt[2]]=uartInt;
      }else if (usbMode){
        //Serial.println("Sending back this Message to USB :");
        for (int i=0;i<(*posInArray);i++){
         // Serial.println(rcvdPkt[i]);
        }
      }
      statusPckt.push_back(rcvdPkt[5]);
      statusPckt.push_back(rcvdPkt[2]);
      statusPckt.push_back(rcvdPkt[6]);
      readStatusPckt(&statusPckt);

      event->setRxBufferSizeTrigger(4);
      (*posInArray) = 0;
      sendFunctionPointer();


    }else{
      if ((*posInArray) >= 4){
        (*numOfBytesToRead) = rcvdPkt[3] + 4;
      }
      event->setRxBufferSizeTrigger((*numOfBytesToRead) - (*posInArray));
      if ((event->getRxBufferSizeTrigger() > 100) | (event->getRxBufferSizeTrigger() <= 0)){
        (*sync) = false;
        event->setRxEventHandler(rxResyncFunctionPointer);
        (*posInArray) = 0;
        event->setRxBufferSizeTrigger(1);
      }
    }
  }
}


void rxEvent1(){
  rxEvent(&event1, &txTimer1, &sync1, &posInArray1, &numOfBytesToRead1, rcvdPkt1,1,&resendCounter1, rxResync1, send1);
}

void rxEvent2(){
  rxEvent(&event2, &txTimer2, &sync2, &posInArray2, &numOfBytesToRead2, rcvdPkt2,2,&resendCounter2, rxResync2, send2);
}

void rxEvent3(){
  rxEvent(&event3, &txTimer3, &sync3, &posInArray3, &numOfBytesToRead3, rcvdPkt3,3,&resendCounter3, rxResync3, send3);
}


void rxResync( UartEvent*          event,
              volatile bool*      sync,
              volatile uint16_t*   posInArray,
              volatile uint8_t*   rcvdPkt,
              void (*rxEventFunctionPointer)(void)){
  if (!event->available()) {
    return;
  }
  rcvdPkt[*posInArray] = ((uint8_t) event->read());

  (*posInArray)+=1;
  if ((*posInArray) >= 4){
    if (rcvdPkt[0] == 255 && rcvdPkt[1] == 255){
      (*sync) = true;
      event->setRxBufferSizeTrigger(rcvdPkt[3] + 4);

      event->setRxEventHandler(rxEventFunctionPointer);
    } else {

      rcvdPkt[0] = rcvdPkt[1];
      rcvdPkt[1] = rcvdPkt[2];
      rcvdPkt[2] = rcvdPkt[3];
      (*posInArray)-=1;
    }
  }

}


void rxResync1(){
  rxResync(&event1, &sync1, &posInArray1, rcvdPkt1, rxEvent1);
}

void rxResync2(){
  rxResync(&event2, &sync2, &posInArray2, rcvdPkt2, rxEvent2);
}

void rxResync3(){
  rxResync(&event3, &sync3, &posInArray3, rcvdPkt3, rxEvent3);
}


void send(UartEvent*                      event,
          QueueArray<DynamixelMessage*>*  queue,
          volatile uint8_t*               resendCounter,
          volatile bool*                  busy,
          Vector<uint8_t>*                messagevector){
  if (!queue->isEmpty()){
    *busy=true;
    DynamixelMessage* message=queue->pop();
    message->assemblePacketfromArray(messagevector);
    event->write(messagevector->data(),messagevector->size());
    delete message;
  }else{
    *busy=false;
  }
}

void send1(){
  send(&event1, &queue1, &resendCounter1,&busy1, &messageVector1);
}

void send2(){
  send(&event2, &queue2, &resendCounter2,&busy2, &messageVector2);
}

void send3(){
  send(&event3, &queue3, &resendCounter3,&busy3, &messageVector3);
}


void pushToQueue( QueueArray<DynamixelMessage*>*  queue,
                  DynamixelMessage*               messageToPush,
                  volatile bool*                  busy,
                  void (*sendFunctionPointer)(void)){
  queue->push(messageToPush);
  if(!(*busy)){
    sendFunctionPointer();
  }
}

void pushToQueue1(DynamixelMessage* messageToPush){
    pushToQueue(&queue1,messageToPush,&busy1,send1);
}

void pushToQueue2(DynamixelMessage* messageToPush){
    pushToQueue(&queue2,messageToPush,&busy2,send2);
}

void pushToQueue3(DynamixelMessage* messageToPush){
    pushToQueue(&queue3,messageToPush,&busy3,send3);
}
