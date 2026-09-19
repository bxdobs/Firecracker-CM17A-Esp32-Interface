// firecracker.h
#ifndef FCC_H
#define FCC_H      
#define _DKV1  // ESP32 -> firecracker (fc mod ri pin trace cut to allow powering direct from esp32)

//#include "includes.h"

// Declarations
// FireCracker config values
#define _FC_HDR_16      0xD5AA // Frame Header 16 bits ... 1101 0101 1010 1010
#define _FC_FTR_8         0xAD // Frame Footer 8 bits  ... 1010 1101
#define _FC_DTR    GPIO_NUM_18 // Actual GPIO to drive the CM17A DTR Pin 4 (DA9)
#define _FC_RTS    GPIO_NUM_19 // Actual GPIO to drive the CM17A RTS Pin 7 (DA9)
#ifdef _DKV1
#define _FC_PWR    GPIO_NUM_23 // allows direct connect VDD to be turned off/on
#endif                         // GPIO 23 controls GND to CM17A via a 2N7000
// _FC_LO = 1 intentionally inverted so 2N3904 provices a LOW output
#define _FC_LO               1 // turns 2N3904 on => Low output on Collector
// _FC_HI = 0 intentionally inverted so 2N3904 provices a HIGH output
#define _FC_HI               0 // turns 2N3904 off => HIGH via Pullup Resistor
#define _FC_FRAME_REPEATS    2 // number of times to repeat FireCracker frame
#define _FC_BIT_TIME_US   1000 // Bit Duration in uSec
#define _FC_GAP_TIME_US    500 // Bit Gap Duration in uSec
#define _FC_FRAME_TIME_MS  800 // Inter Repeat Delay in mSec
#define _FC_START_TIME_MS  500
#define _FC_STOP_TIME_MS  2000
#define _FC_RESET_TIME_MS   50
#define _FC_WAKE_TIME      300 // 30 seconds * .1 Sec 
#define _FC_ON               1
#define _FC_OFF              0

class FirecrackerClass {
public:
   struct FC_Parms {
      int hdr   = _FC_HDR_16;
      int ftr   = _FC_FTR_8; 
gpio_num_t rts  = _FC_RTS;
gpio_num_t dtr  = _FC_DTR;
#ifdef _DKV1      
gpio_num_t pwr  = _FC_PWR;
#endif      
      int frRpt = _FC_FRAME_REPEATS;
      int bitTm = _FC_BIT_TIME_US;
      int gapTm = _FC_GAP_TIME_US;
      int frmTm = _FC_FRAME_TIME_MS;
      int strTm = _FC_START_TIME_MS;
      int stpTm = _FC_STOP_TIME_MS;
      int rstTm = _FC_RESET_TIME_MS;
      int wkTm  = _FC_WAKE_TIME;
      bool fcOn = false;
   } fcp;

   static const char * const zones[];
   static const uint16_t unitCodes[];
   static const char * const units[];
   static const uint8_t cmdCodes[];
   static const char * const cmds[];
   
   void begin();
   void loop();
   
   // hardware routines
   bool pwrFC(int pwrState);
   String dcfc(uint16_t code);
   void resetFC(void);

   void initFCgpio(void);
 
   int  txBitStream(const uint8_t bits[40]);
   uint16_t str2hex(const String& hexstr);

   void firecracker(uint16_t cmd16);

   // dq tasks
   static void dqFirecrackerTask(void *pv);
   static void dqMqttPubTask(void *pv);
 
   // inter Core handlers
   bool wsHandler(JsonDocument& doc);
   //bool httpHandler(const EspCoreClass::EndPoint& epp);
   String EndPointExtension(const EspCoreClass::EndPoint& epp);
   bool mqttHandler(const EspCoreClass::MqttMsg& M);

};
// ---- GLOBAL INSTANCE ----
extern FirecrackerClass fcc;

#endif // FCC_H
