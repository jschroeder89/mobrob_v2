#include "DynamixelController.hpp"

uint8_t rcvdPktUsb[MAX_LENGTH_OF_MESSAGE];
uint8_t posInArrayUsb;

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

volatile uint8_t idMap[256];
volatile bool usbMode=false;
volatile bool scanMode=false;

void scanPort() {
    scanMode = true;

    for (int i=0;i<254;i++){
        idMap[i]=0;
    }
    uint8_t search[16];
    for(int i=0;i<16;i++)
    {
        search[i]=0;
    }

    for(int k=0;k<6;k++){
        for (int j=32*k;j<32*(k+1);j++){
            search[0]=255;
            search[1]=255;
            search[2]=j;
            search[3]=4;
            search[4]=2;
            search[5]=3;
            search[6]=1;
            DynamixelMessage* ScanMessage1 = new DynamixelMessage(search); //messages are being generated and put
            DynamixelMessage* ScanMessage2 = new DynamixelMessage(search); //
            DynamixelMessage* ScanMessage3 = new DynamixelMessage(search); //

            QueueArray <DynamixelMessage*> queue1(50);
            QueueArray <DynamixelMessage*> queue2(50);
            QueueArray <DynamixelMessage*> queue3(50);

            queue1.push(ScanMessage1);                                                   //messages are being pushed
            queue2.push(ScanMessage2);                                                   //into their corresponding queues
            queue3.push(ScanMessage3);

        }

        send1();
        send2();
        send3();
        while(busy1 && busy2 && busy3){
            delay(10);                                                                 //Waiting for ports to finish writing all messages
        }
        scanMode=false;
    }
    Serial.println("DONE!");
}

void init() {
    Uart1Event event1;//initialize UART A of the Teensy for enhanced features like DMA capability
    Uart2Event event2;//initialize UART B ""
    Uart3Event event3;//initialize UART C ""

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

    delay(4000);
    scanPort();
    delay(2000);
    Serial.println("UART Initialization complete!");
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
    DynamixelMessage* USBMessage = new DynamixelMessage(servoPckt);

    if (idMap[servoPckt[2]] == 1 || scanMode) {
        pushToQueue1(USBMessage);
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

void readStatusPckt(Vector<int>* statusPckt) {
    if (statusPckt->at(0) != 0) {
        convertToReadableVelocities(statusPckt);
    }
}


void convertToReadableVelocities(Vector<int>* servoPckt) {
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
    uint8_t servoPckt[8] {FF, FF, 0, _READ_LENGHT, _READ_SERVO_DATA,
        SERVO_REGISTER_PRESENT_SPEED, _NUM_OF_BYTES_TO_READ, 0};
    for (uint8_t id = 1; id <=2; id++) {
        uint8_t checkSum = 0;
        servoPckt[2] = id;
        for (size_t i = 2; i < sizeof servoPckt-1; i++) {
            checkSum += servoPckt[i];
        }
        checkSum = ~(checkSum) & 255;
    servoPckt[sizeof servoPckt-1] = checkSum;

    Serial.print("ServoPacket for ID: ");
    Serial.println(id);
    for (size_t i = 0; i < sizeof servoPckt; i++) {
        Serial.print(servoPckt[i] );
        Serial.print(" ");
    }
    Serial.println("");
    writeToUART(&servoPckt[0]);
    }
}

void servoWritePcktConstructor(Vector<int>* velArray) {
    Serial.println(velArray->at(0));
    Serial.println(velArray->at(1));
    uint8_t servoPckt[9] {FF, FF, 0, _WRITE_LENGHT, _WRITE_SERVO_DATA,
        SERVO_REGISTER_MOVING_SPEED, 0, 0, 0};
    for (uint8_t id = 1; id <= 2; id++) {
        int val = 0;
        uint8_t checkSum = 0;
        servoPckt[2] = id;
        val = velArray->at(id-1);
        if (val < 0) {
            val *= (-1);
            val += 1024;
        }
        servoPckt[6] = val & 255;
        servoPckt[7] = val >> 8;
            for (size_t i = 2; i < sizeof servoPckt-1; i++) {
                checkSum += servoPckt[i];
            }
        checkSum = ~(checkSum) & 255;
        servoPckt[sizeof servoPckt-1] = checkSum;


        Serial.print("ServoPacket for ID: ");
        Serial.println(id);
        for (size_t i = 0; i < sizeof servoPckt; i++) {
            Serial.print(servoPckt[i] );
            Serial.print(" ");
        }
        Serial.println("");
        writeToUART(&servoPckt[0]);
    }
    velArray->clear();
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

void requestHandler() {
    while (Serial.available() > 0) {
        char requestByte = Serial.read();
        switch (requestByte) {
            case sensorReadByte:
                Serial.println(sensorReadByte);
                readSensorData();
                break;
            case servoReadByte:
                Serial.println(servoReadByte);
                servoReadPcktConstructor();
                break;
            case servoWriteByte:
                Serial.println(servoWriteByte);
                readFromUSB();
                break;
        }
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
    Uart1Event event1;
    IntervalTimer txTimer1;
  noMessageReceival(&event1, &txTimer1, &resendCounter1,send1, &messageVector1);
}

void noMessageReceival2(){
    Uart2Event event2;
    IntervalTimer txTimer2;
    noMessageReceival(&event2, &txTimer2, &resendCounter2,send2, &messageVector2);
}

void noMessageReceival3(){
    Uart3Event event3;
    IntervalTimer txTimer3;
    noMessageReceival(&event3, &txTimer3, &resendCounter3,send3, &messageVector3);
}


void txEvent( IntervalTimer* timer,                                               //Timer interrupt that triggers a function if a message does not
              void (*noMessageReceivalFunctionPointer)(void)){                    //reply after the time specified in begin(). The timer uses Âµseconds as a unit.
  timer->priority(255);
  timer->begin(noMessageReceivalFunctionPointer,300);
}

void txEvent1(){
    IntervalTimer txTimer1;
  txEvent(&txTimer1, noMessageReceival1);
}

void txEvent2(){
    IntervalTimer txTimer2;
  txEvent(&txTimer2, noMessageReceival2);
}

void txEvent3(){
    IntervalTimer txTimer3;
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
      if(scanMode){
        idMap[rcvdPkt[2]]=uartInt;
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
    Uart1Event event1;
    IntervalTimer txTimer1;
    volatile uint8_t rcvdPkt1[MAX_LENGTH_OF_MESSAGE];
  rxEvent(&event1, &txTimer1, &sync1, &posInArray1, &numOfBytesToRead1, rcvdPkt1,1,&resendCounter1, rxResync1, send1);
}

void rxEvent2(){
    Uart2Event event2;
    IntervalTimer txTimer2;
    volatile uint8_t rcvdPkt2[MAX_LENGTH_OF_MESSAGE];
  rxEvent(&event2, &txTimer2, &sync2, &posInArray2, &numOfBytesToRead2, rcvdPkt2,2,&resendCounter2, rxResync2, send2);
}

void rxEvent3(){
    Uart3Event event3;
    IntervalTimer txTimer3;
    volatile uint8_t rcvdPkt3[MAX_LENGTH_OF_MESSAGE];
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
    Uart1Event event1;
    volatile uint8_t rcvdPkt1[MAX_LENGTH_OF_MESSAGE];
  rxResync(&event1, &sync1, &posInArray1, rcvdPkt1, rxEvent1);
}

void rxResync2(){
    Uart2Event event2;
    volatile uint8_t rcvdPkt2[MAX_LENGTH_OF_MESSAGE];
  rxResync(&event2, &sync2, &posInArray2, rcvdPkt2, rxEvent2);
}

void rxResync3(){
    Uart3Event event3;
    volatile uint8_t rcvdPkt3[MAX_LENGTH_OF_MESSAGE];
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
    Uart1Event event1;
    QueueArray <DynamixelMessage*> queue1(50);
  send(&event1, &queue1, &resendCounter1,&busy1, &messageVector1);
}

void send2(){
    Uart2Event event2;
    QueueArray <DynamixelMessage*> queue2(50);
  send(&event2, &queue2, &resendCounter2,&busy2, &messageVector2);
}

void send3(){
    Uart3Event event3;
    QueueArray <DynamixelMessage*> queue3(50);
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
    QueueArray <DynamixelMessage*> queue1(50);
    pushToQueue(&queue1,messageToPush,&busy1,send1);
}

void pushToQueue2(DynamixelMessage* messageToPush){
    QueueArray <DynamixelMessage*> queue2(50);
    pushToQueue(&queue2,messageToPush,&busy2,send2);
}

void pushToQueue3(DynamixelMessage* messageToPush){
    QueueArray <DynamixelMessage*> queue3(50);
    pushToQueue(&queue3,messageToPush,&busy3,send3);
}
