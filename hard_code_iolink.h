/*
	Copyright (C) 2021 unref-ptr
    This file is part of Hardcoded IO-Link.

    Hardcoded IO-Link is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Hardcoded IO-Link is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Hardcoded IO-Link.  If not, see <https://www.gnu.org/licenses/>.
*/

/* IO-Link COM Speeds*/
#define IO_LINK_COM3 230400
#define IO_LINK_COM2 38400
#define IO_LINK_COM1 4800
#define MASTER_READ 1

/* IO-Link spec definitions*/
//position where the master ckt frame is located (starting at 0)
#define CKT_FRAME 1 
//Types of IO-Link channels according Table A.1
#define PROCESS 0
#define PAGE 1
#define DIAGNOSIS 2
#define ISDU 3
/* Hard coded IO-Link device definitions*/
#define SIZE_OD_PREOPERATE 8
#define SIZE_OD_OPERATE 8
#define SIZE_PDOUT 2
#define SIZE_PDIN 0

/*
Delay required when sending the data
as arduino serial is non-blocking for TX
*/
#define COM2_DELAY 286 //(11 bits)/COM2  = 286 us
//Modes for the Device
typedef enum ioLink_mode 
{
	start,
	preoperate,
	operate
} ioLink_mode_e;
//States for interpreting IO-Link data
typedef enum ioLink_states
{	
	wait_wake, //Waiting for wakeup signal
	wait_valid_frame, //Waiting for a complete IO-Link Master frame
	run_mode	//state where checking for a new Master frame and parsing 
				//data reciveed
} ioLink_states_e;

//HW connections for the IO-Link Transciever TIOL-111
const uint8_t enablePin = 4;
const uint8_t wakeupPin = 2;
uint8_t ignore_RX_bytes = 0;
//Message struct for IO-Link
typedef struct {
  uint8_t channel;
  uint8_t addr;
  bool read;
} ioLink_message_t;

ioLink_message_t message;

//Max buffer for IO-Link data [1 byte MC + 1 Byte CKT + 32 PD + 32 OD]
#define MAX_BUFFER_IOLINK 66
//Direct Parameter Page 1 values

//Hard coded Direct Parameter Page 1 Values
uint8_t paramterPage1[16] =
{
  0,
  0,
  0x86, //128 ms cycletime
  0x2C, //8 preo 8 op isdu unsup
  0x11, //Revision ID
  0, //PDIn
  16, //PDOut 2 bytes
  0xAB, //Vendor ID
  0xCD,
  0,
  0,
  1,
  0,
  0,
  0
};
//Hard coded checksums for Direct Parameter Page 1
//Check spec section A.1.3
uint8_t ckt_pp1[15] =
{
  0x2d,
  0x2d,
  0x36,
  0x39,
  0x28,
  0x2d,
  0x39,
  0x33,
  0x3c,
  0x2d,
  0x2d,
  0x3c,
  0x2d,
  0x2d,
  0x2d
};
volatile bool wakeUp_started = false;
void wakeup() {
  wakeUp_started = true;
}

class hardcode_iolink
{
  public:
    void init(Stream &serial);
    void task();
    bool getPDOut(uint8_t * buffer);
  private:
    Stream* _serial;
    uint8_t pdOut[SIZE_PDOUT];
    void initTransciever();
    void decodeMessage();
    void sendOK();
    void send_ioLink_data(uint8_t *data, uint8_t len);
    bool new_ioLink_byte();
    ioLink_mode_e deviceMode = start;
    ioLink_states_e deviceState = wait_wake;
    uint8_t rxBuffer[MAX_BUFFER_IOLINK];
    uint8_t txBuffer[MAX_BUFFER_IOLINK];
    uint8_t byteCount = 0;
    uint8_t MC_xWR = 0;
    uint8_t maxFrames = 0;
    bool newIOLinkMasterFrame_recieved = false;
    uint8_t read_ioLink_byte();
    void prepareMessage(uint8_t od_size);
    void checkNewIOLinkMaster_Frame (uint8_t NewByte) ;
};
bool hardcode_iolink::getPDOut(uint8_t * buffer)
{
  memcpy(buffer, pdOut, SIZE_PDOUT);
  if (deviceMode == operate)
  {
    return true;
  }
  else
  {
    return false;
  }
}
void hardcode_iolink::prepareMessage(uint8_t od_size)
{
  switch (message.channel)
  {
    case PAGE:
      if (!message.read)
      {
        if (rxBuffer[2] == 0x99) //Operate mode 
        {
          deviceMode = operate;
        }
        sendOK();
      }
      else
      {
		  //Send the harcoded direct parameter page 1 value
        txBuffer[0] = paramterPage1[message.addr];
        txBuffer[od_size] = ckt_pp1[message.addr];
        memset(txBuffer + 1, 0, od_size - 1);
        send_ioLink_data(txBuffer, od_size + 1);
      }
      break;
    case ISDU:
	  //Hardcoded IO-Link, no chance for ISDU 
      memset(txBuffer, 0, od_size); //No service
      txBuffer[od_size] = 0x2D;
      send_ioLink_data(txBuffer, od_size + 1);
      break;
  }
  //if in operate mode get the PdOut
  //No Pdin available at the moment
  if (deviceMode == operate)
  {
    memcpy(pdOut, &rxBuffer[2], SIZE_PDOUT);
  }
}

//Send confirmation when IO-Link master does a
//write request
void hardcode_iolink::sendOK()
{
  txBuffer[0] = 0x2D;
  send_ioLink_data(txBuffer, 1);
}
//Decode the data from the IO-Link master frame MC
void hardcode_iolink::decodeMessage()
{
  message.channel = (rxBuffer[0] & 0x60) >> 5;
  message.addr = rxBuffer[0] & 0x1F;
  message.read = rxBuffer[0] >> 7;
}
//Initialize transciever
void hardcode_iolink::init(Stream &serial)
{
  initTransciever();
  _serial = &serial;
}
// check for new IO-Link master frames
void hardcode_iolink::checkNewIOLinkMaster_Frame (uint8_t NewByte)  
{
  rxBuffer[byteCount] = NewByte;
  if (byteCount == 0)
  {
    MC_xWR = rxBuffer[0] >> 7; //First Uart frame is MC, check IO-Link spec  Figure A.1
  }
  byteCount ++;
  if (byteCount == CKT_FRAME )
  {
	//Get MSeqType Bits (Figure A.2  IO-Link Spec)
    uint8_t mSeqType = rxBuffer[CKT_FRAME ] >> 6; 
    if (mSeqType == 0) //Startup
    {
      if (MC_xWR == MASTER_READ) //If read
      {
        maxFrames = 2;
      }
      else
      {
        maxFrames = 3;
      }
    }
	//Preoperate
    else if (mSeqType == 1) 
    {
      if (MC_xWR == MASTER_READ)
      {
        maxFrames = 2;
      }
      else
      {
        maxFrames = SIZE_OD_PREOPERATE + 2;
      }
    }
	//Operate mode
    else if (mSeqType == 2) 
    {
      if (MC_xWR == MASTER_READ)
      {
        maxFrames = 2 + SIZE_PDOUT;
      }
      else
      {
        maxFrames = 2 + SIZE_OD_OPERATE + SIZE_PDOUT;
      }
    }
  }
  //this is the flag that should be sent as notification for this event
  if (byteCount == maxFrames)
  {
    newIOLinkMasterFrame_recieved = true; 
  }
}
/*As the IO-Link Transciever
*Sends what its recieve, it must
* ignore the same quantity of bytes
* it sends
*/
bool hardcode_iolink::new_ioLink_byte()
{
  if (_serial->available() > 0)
  {
    if (ignore_RX_bytes == 0)
    {
      return true;
    }
    else
    {
      Serial.read();
      ignore_RX_bytes -= 1;
      return false;
    }
  }
  else
  {
    return false;
  }
}
//Wrapper for Serial HW
uint8_t hardcode_iolink::read_ioLink_byte()
{
  return _serial->read();
}

//HW Setup
void hardcode_iolink::initTransciever()
{
  pinMode(enablePin, OUTPUT);
  pinMode(wakeupPin, INPUT);
  digitalWrite(enablePin, HIGH);
  //Wakeup is detected with flaling edge
  attachInterrupt(digitalPinToInterrupt(wakeupPin), wakeup, FALLING );
}

//While sending data the enable line must be held high
//As arduino serial is non-blocking, must wait_valid_frame
//a hardcoded value corresponding to the time it takes to send the data
void hardcode_iolink::send_ioLink_data(uint8_t *data, uint8_t len)
{
  byteCount = 0;
  maxFrames = 0xFF;
  ignore_RX_bytes = len;
  digitalWrite(enablePin, HIGH);
  for (uint8_t i = 0; i < len; i++)
  {
    _serial->write(*(data + i));
  }
  delayMicroseconds(COM2_DELAY * len);
  digitalWrite(enablePin, LOW);
}
//Main task to process IO-Link data
void hardcode_iolink::task()
{
  switch (deviceState)
  {
    case wait_wake:
      if (wakeUp_started)
      {
        wakeUp_started = false;
        deviceState = wait_valid_frame;
        digitalWrite(enablePin, LOW);
      }
      break;
    case wait_valid_frame:
      {
        if (new_ioLink_byte())
        {
          uint8_t rx_byte = read_ioLink_byte();
          if (rx_byte == 0xA2)
          {
            deviceState = run_mode;
            checkNewIOLinkMaster_Frame(rx_byte);
          }
        }
      }
      break;
    case run_mode:
      if (new_ioLink_byte())
      {
        checkNewIOLinkMaster_Frame(read_ioLink_byte());
      }
      if (newIOLinkMasterFrame_recieved)
      {
        newIOLinkMasterFrame_recieved = false;
        if (deviceMode == start)
        {
          if (byteCount == 3)
          {
            if (rxBuffer[2] == 0x9A)
            {
              deviceMode = preoperate;
            }
            sendOK();
          }
          else
          {
            uint8_t address = rxBuffer[0] & 0x0F;
            txBuffer[0] = paramterPage1[address];
            txBuffer[1] =  ckt_pp1[address];
            send_ioLink_data(txBuffer, 2);
          }
        }
        else if (deviceMode == preoperate)
        {
          decodeMessage();
          prepareMessage(SIZE_OD_PREOPERATE);
        }
        else if (deviceMode == operate)
        {
          decodeMessage();
          prepareMessage(SIZE_OD_OPERATE);
        }
      }
      break;
  }
}
