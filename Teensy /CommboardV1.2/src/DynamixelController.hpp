#ifndef DynamixelController_HPP
#define DynamixelController_HPP

#include <UartEvent.h>
#include <DynamixelMessage.h>

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
#define MAX_LENGTH_OF_MESSAGE 259


class DynamixelController {

    public:
        bool newData = false;
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
        void UartInit();


        void convertServoDataToJson(int* dataArray);
        void parseJsonString(String s);
        void servoWritePcktConstructor(Vector<int>* velArray);
        void servoReadPcktConstructor(int* servoPckt);
        void writeToUART();
        void convertVelocities(Vector<int>* servoPckt);
        void readStatusPckt(uint8_t* rcvdPkt);
        void readFromUSB();


    private:
        Uart1Event event1;
        Uart2Event event2;
        Uart3Event event3;

        IntervalTimer txTimer1;
        IntervalTimer txTimer2;
        IntervalTimer txTimer3;
        
        uint16_t size = 50;
        QueueArray <DynamixelMessage*> queue1{size};
        QueueArray <DynamixelMessage*> queue2{size};
        QueueArray <DynamixelMessage*> queue3{size};

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

        volatile bool busy1 = false;
        volatile bool busy2 = false;
        volatile bool busy3 = false;

        volatile uint8_t resendCounter1 = 0;
        volatile uint8_t resendCounter2 = 0;
        volatile uint8_t resendCounter3 = 0;

        volatile bool usbMode = false;

        uint8_t rcvdPktUsb[MAX_LENGTH_OF_MESSAGE];
        uint8_t posInArrayUsb=0;

        volatile uint8_t idMap[256];
        volatile bool scanMode = false;
};

#endif
