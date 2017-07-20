#include <UartEvent.h>
#include <DynamixelMessage.h>
#include <QueueArray.h>
#include <Arduino.h>
#include <i2c_t3.h>
#include <time.h>
#include <string>
#include <vector>
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
#define acknowledgeSize 'a'

#define HEADER 0XFF


#define sensorRead 1
#define servoRead 2
#define servoWrite 3
//Arrays
uint8_t segments[5] = {1,2,3,4,5};
char sgmnts[5]= {'A','B','C','D','E'};
char velLeftChars[2]={'L','N'};
char velRightChars[2]={'R','N'};
uint8_t address[] = {AD0, AD1, AD2, AD3, AD4};
uint8_t channel[] = {CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7};
int data[5][8];
int ledPin=13;
int velLeft[2]={0};
int velRight[2]={0};
char incomingByte;

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

volatile bool scanMode=false;           //if scanMode is set to true, the servos that are connected
                                        //will be memorized in the idMap. scanMode is being set
                                        //in the scanPort function automatically


uint8_t rcvdPktUsb[MAX_LENGTH_OF_MESSAGE];
uint8_t posInArrayUsb=0;

volatile uint8_t idMap[256];

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

void setup(){
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
  Serial.begin(9600);
  pinMode(13, LOW);
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


  //delay(4000);
  scanPort();
  //delay(2000);
}

int getSensorData(int add, int ch)
{
  //i2c = Wire;
  //digitalWrite(13, HIGH);

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

void jsonConstructor(int op) {
    StaticJsonBuffer<500> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();

}

void dynaPacketConstructor() {

}

void requestHandler() {
    if (Serial.available() > 0) {
        incomingByte = Serial.read();
        switch (incomingByte) {
            case sensorReadByte:
                jsonConstructor(sensorRead);
            case servoReadByte:
                jsonConstructor(servoRead);
            case servoWrite:
                dynaPacketConstructor();
        }
    }
}

char readJsonString() {
    StaticJsonBuffer<200> jsonBuffer;
    char buf[200], c;
    int index = 0;
    do {
        c = Serial.read();
        buf[index]=c;
        index++;
        delay(100);

        Serial.println(c);
    } while(c!='}');
    buf[index]='\0';
    JsonObject& root = jsonBuffer.parseObject(buf);
    int data = root["servoData"][0];
    return 0;
}

void loop() {
    //StaticJsonBuffer<500> jsonBuffer;
    //JsonObject& root = jsonBuffer.createObject();
    delay(300);
    readJsonString();

        /*switch (incomingByte) {
            case sensorRead:
            if (Serial.availableForWrite() > 0) {
                root["data"] = "sensor";
                JsonArray& sensorF = root.createNestedArray("F");
                JsonArray& sensorR = root.createNestedArray("R");
                JsonArray& sensorL = root.createNestedArray("L");
                JsonArray& sensorB = root.createNestedArray("B");
                sensorF.add(4095);
                sensorF.add(1204);
                sensorL.add(3230);
                sensorR.add(2383);
                sensorB.add(2383);
                root.printTo(Serial);

            }
                break;
                case servoRead:Serial.println(incomingByte);
                    break;
                    case servoWrite:Serial.println(incomingByte);
                        break;
        }*/


    /*while (Serial.availableForWrite()>0) {
        delay(100);
        Serial.write("{test}");
        delay(100);

    }*/
  //rxSerialEventUsb();
  //uint8_t i = 8;
  //uint8_t u = 1;
  //digitalWrite(ledPin, HIGH);
  //delay(50);
    /*for ( i = 0; i < sizeof(segments); i++) {
            Serial.println(sgmnts[i]);
        for ( j = 0; j < sizeof(channel); j++)  {
            data[i][j] = getSensorData(i,j);
            Serial.println(data[i][j]);
        }
    }
    Serial.println(velLeftChars[0]);
    for (int i = 0; i < 2; i++) {
        Serial.println(velLeft[i]);
    }
    Serial.println(velLeftChars[1]);

    Serial.println(velRightChars[0]);
    for (int i = 0; i < 2; i++) {
        Serial.println(velRight[i]);
    }
    Serial.println(velRightChars[1]);
    */

    //Serial.println("test");


}

void scanPort(){

  scanMode=true;
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
void rxSerialEventUsb(){
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
     // Serial.print("Received a Message from :");
      //Serial.println(rcvdPkt[2]);
      //Serial.println("Packet is:");
          //digitalWrite(ledPin, HIGH);
          //if (rcvdPkt[4]==2 && rcvdPkt[5]==38) {



              //digitalWrite(ledPin, LOW);
          //}
      //for(int i =0;i<rcvdPkt[3]+4;i++){
        //Serial.println(rcvdPkt[i]);
      //}
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
      if(scanMode){
        idMap[rcvdPkt[2]]=uartInt;
      }else if (usbMode){
        //Serial.println("Sending back this Message to USB :");
        for (int i=0;i<(*posInArray);i++){
         // Serial.println(rcvdPkt[i]);
        }
      }
      if (rcvdPkt[2]==1) {
          velLeft[0]=rcvdPkt[5];
          velLeft[1]=rcvdPkt[6];
      }
        if (rcvdPkt[2]==2) {
            velRight[0]=rcvdPkt[5];
            velRight[1]=rcvdPkt[6];
        }

      /*Serial.println(pcktSegStrt);
          Serial.println(rcvdPkt[5]);
          Serial.println(rcvdPkt[6]);
      Serial.println(pcktSegEnd );*/
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
