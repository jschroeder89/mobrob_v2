#include "DynamixelMessage.h"



DynamixelMessage::DynamixelMessage(uint8_t id, uint8_t length, uint8_t messageType, uint8_t reg, uint8_t value)
{
  //When calling the constructor, all given variables are written to private variables in the class.
    DynamixelMessage::_id=id;
    DynamixelMessage::_messageType=messageType;
    DynamixelMessage::_reg=reg;
    DynamixelMessage::_value=value;
}
DynamixelMessage::DynamixelMessage(uint8_t* usbPacket)
{
  this->_id=usbPacket[2];
  this->_length=usbPacket[3];
  this->_messageType=usbPacket[4];
  this->_reg=usbPacket[5];
  for (int k=0;k<(usbPacket[3]-3);k++)
  {
    this->_payload[k]=usbPacket[k+6];
  }
}

void DynamixelMessage::assemblePacketfromArray(Vector<uint8_t>* assembledPacket)
{
  assembledPacket->clear();
  uint8_t checksumResult =0;


  assembledPacket->push_back(0XFF);
  assembledPacket->push_back(0XFF);
  assembledPacket->push_back(_id);
  assembledPacket->push_back(_length);
  assembledPacket->push_back(_messageType);
  assembledPacket->push_back(_reg);

  for(int j=0;j<_length-3;j++)
  {
    assembledPacket->push_back(_payload[j]);
  }

  for(int p=2;p<=assembledPacket->at(3)+2;p++)
  {

      checksumResult=checksumResult+assembledPacket->at(p);
  }
  checksumResult=~(checksumResult)&255;

  assembledPacket->push_back(checksumResult);

}

void DynamixelMessage::assemblePacket(Vector<uint8_t>* assembledPacket)
//This function will assemble a packet correctly so that it can be pushed to the queue, and sent to a Dynamixel Servo-Drive.
//The function will use the private variables that were given when calling the constructor
//of the class to assemble the packet.
//IMPORTANT NOTE: assemblePacket() is currently implemented for the regular Dynamixel Series only and will not work with the Dynamixel Pro Series.
{
    assembledPacket->clear();
    uint8_t pkt[255];
    uint8_t checksumResult =0;

    for (int t = 0; t < 255; t++)
    {
        pkt[t] = 0;
    }

    pkt[0] = 0XFF;
    pkt[1] = 0XFF;
    pkt[2] = DynamixelMessage::_id;
    pkt[3] = DynamixelMessage::_length;
    //The length is determined as "number of parameters(N) +2". For the case of just reading one byte from one register this will result in
    // a length of 4 (register to read from (1st parameter) + payload of how many bytes to read from that register on (2nd parameter) + 2 )
    // This needs to be adjusted for reading more than one byte
    //Determining what type of message(READ,WRITE,SYNCWRITE) to send to the Dynamixel.
    pkt[4]=DynamixelMessage::_messageType;
    pkt[5] =DynamixelMessage::_reg;
    pkt[6] =DynamixelMessage::_value;

    //checksum calculation = The checksum is calculated by bit-wise inverting the sum of all parameters from the message except for the
    //first two bytes ;
    for(int p=2;p<=pkt[3]+2;p++)
    {

        checksumResult=checksumResult+pkt[p];
    }
    checksumResult=~(checksumResult)&255;

    pkt[pkt[3]+3]=checksumResult;

    //Pushing the message into the vector pointer that got passed when calling
    //the function.
    //This will dynamically allocate the space in the Vector in main.cpp depending on how big pkt[3] is e.g. how big the packet is.
    //Serial.println("Beginning of Message");
    for (int j=0;j<pkt[3]+4;j++)
    {
      //Serial.println(assembledPacket->at(j));
      assembledPacket->push_back(pkt[j]);
    }

}


//Definition of Getter-/Setter-Methods for the private variables
uint8_t DynamixelMessage::get_id() const {
    return _id;
}

void DynamixelMessage::set_id(uint8_t _id) {
    DynamixelMessage::_id = _id;
}

uint8_t DynamixelMessage::get_length() const {
    return _length;
}

void DynamixelMessage::set_length(uint8_t _length) {
    DynamixelMessage::_length = _length;
}

uint8_t DynamixelMessage::get_reg() const {
    return _reg;
}

void DynamixelMessage::set_reg(uint8_t _reg) {
    DynamixelMessage::_reg = _reg;
}

uint8_t DynamixelMessage::get_value() const {
    return _value;
}

void DynamixelMessage::set_value(uint8_t _value) {
    DynamixelMessage::_value = _value;
}
