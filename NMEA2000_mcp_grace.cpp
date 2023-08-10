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
#include "esp_log.h"

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
static const char* TAG = "MCP";
//struct tCANFrame {
//  uint32_t id; // can identifier
//  uint8_t len; // length of data
//  uint8_t buf[8];
//};

bool CanInUse=false;
tNMEA2000_mcp *pNMEA2000_mcp1=0;

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst);
#if defined(ESP8266)
ICACHE_RAM_ATTR void Can1Interrupt();
#else
void Can1Interrupt();
#endif

//*****************************************************************************
//void PrintDecodedCanIdAndLen(unsigned long id, unsigned char len) {
//  unsigned char prio;
//  unsigned long pgn;
//  unsigned char src;
//  unsigned char dst;
//
//  if (id!=0) {
//    CanIdToN2k(id,prio,pgn,src,dst);
//    Serial.print(millis());
//    Serial.print(": pgn: "); Serial.print(pgn); Serial.print(", prio: "); Serial.print(prio);
//    Serial.print(", src: "); Serial.print(src); Serial.print(", dst: "); Serial.print(dst);
//  } else {
//    Serial.print("id: "); Serial.print(id);
//  }
//  Serial.print(", len: "); Serial.println(len);
//}

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
  //tFrameBuffer *pTxBuf=0;
    ESP_LOGD(TAG, "ID to send: %lx",id);
    ESP_LOGD(TAG, "DLC to send: %u",len);
    ESP_LOGD(TAG, "DATA[0] to send: %u",buf[0]);
        struct can_frame frame;
        frame.len = len>8?8:len;
        memcpy(frame.buf, buf, 8);
        frame.id = id + 0x80000000; //SPI driver requires MSB to be set
        result = N2kCAN.sendMessage(&frame);
        ESP_LOGD(TAG, "CANSendFrame called");
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

//*****************************************************************************
bool tNMEA2000_mcp::CANOpen() {
    if (IsOpen) return true;

    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

    //N2kCAN.reserveTxBuffers(1); // Reserve one buffer for fast packet.
    IsOpen=(N2kCAN.begin(N2k_CAN_CS_pin)==ESP_OK && N2kCAN.setBitrate(CAN_250KBPS,MCP_8MHZ)==ESP_OK );

    N2kCAN.setNormalMode();

//    if (IsOpen && UseInterrupt() ) {
//#ifdef USE_SREG
//    uint8_t SaveSREG = SREG;   // save interrupt flag
//#endif
//      //noInterrupts();
//      N2kCAN.enableTxInterrupt();
//      attachInterrupt(digitalPinToInterrupt(N2k_CAN_int_pin), Can1Interrupt, FALLING);
//      InterruptHandler(); // read out possible data to clear isr bit.
//#ifdef USE_SREG
//      SREG = SaveSREG;   // restore the interrupt flag
//#else
//      //interrupts();
//#endif  
//    }

    CanInUse=IsOpen;

    return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_mcp::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
struct can_frame frame;
    if ( UseInterrupt() ) {

      HasFrame=pRxBuffer->GetFrame(id,len,buf);

      //ESP_LOGD("CANgetFrame: ", "use interupt");
    } else {
      if ( N2kCAN.readMessage(&frame) == MCP2515::ERROR_OK) {           // check if data coming
          //struct can_frame frame;
          //N2kCAN.readMessage(&frame);    // read data,  len: data length, buf: data buf
          id = frame.id;
          len = frame.len;
          memcpy(buf, frame.buf, len);
          //id = N2kCAN.getCanId();
          //ESP_LOGD("CANgetFrame: ", "ID: %lx, DATA[0] %x", id, buf[0]);
          HasFrame=true;
      }
    }

    // if (HasFrame) PrintDecodedCanIdAndLen(id,len);
    //ESP_LOGD("CANgetFrame: ", "called");
    
    //ESP_LOGI("CANgetFrame: ", "called");
    //ESP_LOGI("CANgetFrame: ", "called");
    
    return HasFrame;
}

//*****************************************************************************
// I am still note sure am I handling volatile right here since mcp_can has not
// been defined volatile. see. http://blog.regehr.org/archives/28
// In my tests I have used only to receive data or transmit data but not both.
//void tNMEA2000_mcp::InterruptHandler() {
//#if defined(DEBUG_NMEA2000_ISR)
//  unsigned long ISRStart=micros();
//#endif
//  uint8_t RxTxStatus;
//
//  // Iterate over all pending messages.
//  // If either the bus is saturated or the MCU is busy, both RX buffers may be in use and
//  // reading a single message does not clear the IRQ conditon.
//  // Also we need to check and clear all transmit flags to clear IRQ condition.
//  // Note that this handler expects that Wakeup and Error interrupts has not been enabled.
//  do {
//    RxTxStatus=N2kCAN.readRxTxStatus();  // One single read on every loop
//    uint8_t tempRxTxStatus=RxTxStatus;      // Use local status inside loop
//    uint8_t status;
//    can_frame *frame;
//
//    while ( (status=N2kCAN.checkClearRxStatus(&tempRxTxStatus))!=0 ) {           // check if data is coming
//      if ( (frame=pRxBuffer->GetWriteFrame())!=0 ) {
//        uint8_t ext,rtr;
//        N2kCAN.readMessage(&(frame->id),&(frame->len),frame->buf);
//        pRxBuffer->IncWrite();
////      asm volatile ("" : : : "memory");
//        //N2kCAN.readMsgBuf(&len,buf);
//        //id=N2kCAN.getCanId();
//        //pRxBuffer->AddFrame(id,len,buf);
//      } else { // Buffer full, skip frame
//        can_frame FrameToSkip;
//        uint8_t ext,rtr;
//        N2kCAN.readMessage(&(FrameToSkip.id),&(FrameToSkip.len),FrameToSkip.buf);
//      }
//    }
//
//    if ( !pTxBufferFastPacket->IsEmpty() ) { // Do we have something to send on fast packet frame buffer
//      // CanIntChk=tempRxTxStatus;
//      if ( (status=N2kCAN.checkClearTxStatus(&tempRxTxStatus,N2kCAN.getLastTxBuffer()))!=0 ) {
//        frame=pTxBufferFastPacket->GetReadFrame();
//        N2kCAN.sendMessage(frame->id, frame->len, frame->buf);
//        pTxBufferFastPacket->DecRead();
//      }
//    } else { // Nothing to send, so clear flag for this buffer
//      status=N2kCAN.checkClearTxStatus(&tempRxTxStatus,N2kCAN.getLastTxBuffer());
//      //N2kCAN.clearBufferTransmitIfFlags(status);TODO
//    }
//
//    if ( !pTxBuffer->IsEmpty() ) { // Do we have something to send on single frame buffer
//      while ( (status=N2kCAN.checkClearTxStatus(&tempRxTxStatus))!=0 && 
//              (frame=pTxBuffer->GetReadFrame())!=0 ) {
//        N2kCAN.sendMessage(frame->id, frame->len, frame->buf);
//        pTxBuffer->DecRead();
//      }
//    } 
//
//    // Finally clear rest transmit flags
//    //N2kCAN.clearBufferTransmitIfFlags(tempRxTxStatus); TODO
//
//  } while ( RxTxStatus!=0 );
//  
//#if defined(DEBUG_NMEA2000_ISR)
//  ISRElapsed=micros()-ISRStart;
//#endif
//}

//#if defined(DEBUG_NMEA2000_ISR)
////*****************************************************************************
//void tNMEA2000_mcp::TestISR() {    // if ( CanIntChk ) { Serial.print("CAN int chk: "); Serial.println(CanIntChk); CanIntChk=0; }
//    if ( ISRElapsed ) { Serial.print("ISR Elapsed: "); Serial.println(ISRElapsed); ISRElapsed=0; }
//}
//#endif


//*****************************************************************************
//#if defined(ESP8266)
//ICACHE_RAM_ATTR void Can1Interrupt() {
//#else
//void Can1Interrupt() {
//#endif
//  pNMEA2000_mcp1->InterruptHandler();
//}