/*
NMEA2000_mcp.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "NMEA2000_mcp.h"

#if defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_UNO)
#define USE_SREG 1
#endif


#if defined(DEBUG_MCP_CAN_SPEED)
  unsigned long McpElapsed=0;
  unsigned long McpStart;
# define DbgStartMcpSpeed McpStart=micros()
# define DbgEndMcpSpeed McpElapsed=micros()-McpStart
# define DbgTestMcpSpeed if ( McpElapsed>0 )
# define DbgClearMcpSpeed McpElapsed=0
//# define DbgPrintN2kMcpSpeed(fmt, args...)     Serial.print (fmt , ## args)
//# define DbgPrintLnN2kMcpSpeed(fmt, args...)   Serial.println (fmt , ## args)
#else
# define DbgPrintN2kMcpSpeed(fmt, args...)
# define DbgPrintLnN2kMcpSpeed(fmt, args...)
# define DbgStartMcpSpeed
# define DbgEndMcpSpeed
# define DbgTestMcpSpeed
# define DbgClearMcpSpeed
#endif
static const char* TAG = "NMEA2000_mcp_grace.cpp";


bool CanInUse=false;
tNMEA2000_mcp *pNMEA2000_mcp1=0;

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
#if defined(ESP8266)
ICACHE_RAM_ATTR void Can1Interrupt();
#else
void Can1Interrupt();
#endif

//*****************************************************************************
tNMEA2000_mcp::tNMEA2000_mcp( spi_device_handle_t *s, unsigned char _N2k_CAN_CS_pin, unsigned char _N2k_CAN_clockset,
                             unsigned char _N2k_CAN_int_pin, uint16_t _rx_frame_buf_size) : tNMEA2000(), N2kCAN(s) {

  IsOpen=false;
  N2k_CAN_CS_pin=_N2k_CAN_CS_pin;
  N2k_CAN_clockset=_N2k_CAN_clockset;
  if (pNMEA2000_mcp1==0) { // Currently only first instance can use interrupts.
    N2k_CAN_int_pin=_N2k_CAN_int_pin;
    if ( UseInterrupt() ) {
      MaxCANReceiveFrames=_rx_frame_buf_size;
      pNMEA2000_mcp1=this;
    }
  } else {
    N2k_CAN_int_pin=0xff;
  }


}

//*****************************************************************************
bool tNMEA2000_mcp::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool wait_sent) {
        MCP2515::ERROR result;
        struct can_frame frame;
        frame.len = len>8?8:len;
        memcpy(frame.buf, buf, 8);
        frame.id = id | 0x80000000; //SPI driver requires MSB to be set
        result = N2kCAN.sendMessage(&frame);
        if (result ==  0) {
          return true;
        }
    return false;
}

//*****************************************************************************
void tNMEA2000_mcp::InitCANFrameBuffers() {
    if ( UseInterrupt() ) {
      if (MaxCANReceiveFrames<2 ) MaxCANReceiveFrames=2;
      if (MaxCANSendFrames<10 ) MaxCANSendFrames=10;
      uint16_t CANGlobalBufSize=MaxCANSendFrames-4;
      MaxCANSendFrames=4;  // we do not need much libary internal buffer since driver has them.
      uint16_t FastPacketBufferSize= (CANGlobalBufSize * 9 / 10);
      CANGlobalBufSize-=FastPacketBufferSize;
      pRxBuffer=new tFrameBuffer(MaxCANReceiveFrames);
      pTxBuffer=new tFrameBuffer(CANGlobalBufSize);
      pTxBufferFastPacket=new tFrameBuffer(FastPacketBufferSize);
    }

    tNMEA2000::InitCANFrameBuffers(); // call main initialization
}
bool tNMEA2000_mcp::CANinit(){

  if(N2kCAN.init_SPI()==ESP_OK){return true;};
  return false;
}
//*****************************************************************************
bool tNMEA2000_mcp::CANOpen() {
    if (IsOpen){
      return true;
    }

    IsOpen=(N2kCAN.begin(N2k_CAN_CS_pin)==ESP_OK && N2kCAN.setBitrate(CAN_250KBPS,MCP_8MHZ)==ESP_OK );

    N2kCAN.setNormalMode();

    CanInUse=IsOpen;

    return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_mcp::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
struct can_frame frame;
    if ( UseInterrupt() ) {

      HasFrame=pRxBuffer->GetFrame(id,len,buf);

    } else {
      if ( N2kCAN.readMessage(&frame) == MCP2515::ERROR_OK) {           // check if data coming
          id = frame.id;
          len = frame.len;
          memcpy(buf, frame.buf, len);
          HasFrame=true;
      }
    }
    
    return HasFrame;
}

