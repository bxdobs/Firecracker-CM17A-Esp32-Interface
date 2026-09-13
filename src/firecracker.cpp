// firecracker.cpp

#include "espcore.h"
#include "firecracker.h"

// 12/09/2026 Note: this sketch uses the protocol required by the 12C508A embedded PIC MCU
//                  within the CM17A which expects DTR and RTS manipulations to produce a
//                  40 bit stream representing D5AA X10C AD where X10C is a 16 bit X10 coded
//                  value ... this 40 bit stream is sent out as 1 mS bit width with 500nS 
//                  bit gaps 
//
//                  In my initial Sketch, I left the device powered after esp32 boot but this
//                  appeared to fail when left endlessly powered so this was changed in this 
//                  version back to the CM17A specified methodology of powering prior to tx
//                  This mod now powers up but leaves the cm17a powered for up to 30 seconds
//                  after each transmission
//
//                  Should the 12C508A die ... the rf cct could potentially be driven directly
//                  The 12C508A sends each Byte of the 2 X10 coded bytes as hdr byte comp byte
//                  ie J-6-on: F050 is sent as F0 0F 50 AF (32 bits) 
//                  rf tx format: hdr (hi 9 mS lo 4.5 mS) followed by 32 bits where 
//                  1 is hi 500nS lo 1500nS (approx 2mS)
//                  0 is hi 500nS lo 500nS  (approx 1mS) 
//                  total len of tx is approx 13.5mS + 48mS  ... [16 * (1mS + 2mS)] 
//                  working measured times: 
//                  hdr: (hi 8.193mS lo 4.55mS) 1: (hi 576nS lo 1699nS) 0: (hi 576nS lo 557nS)
//                      
// X10 16 bit codes = ZONE + UNIT + CMD (for ON and OFF) or ZONE + CMD (for all other CMDs)
//	X10	Firecracker    RF CODE
//	ZONE            A	      6000
//	                B	      7000
//	                C	      4000
//	                D	      5000
//	                E	      8000
//	                F	      9000
//	                G	      A000
//	                H	      B000
//	                I	      E000
//	                J	      F000
//	                K	      C000
//	                L	      D000
//	                M	      0000
//	                N	      1000
//	                O	      2000
//	                P	      3000
// UNIT	         01	      0000
//	               02	      0010
//	               03	      0008
//	               04	      0018
//                05	      0040
//	               06	      0050
//	               07	      0048
//	               08	      0058
//	               09	      0400
//	               10	      0410
//	               11	      0408
//	               12	      0418
//             	13	      0440
//	               14	      0450
//	               15	      0448
//	               16	      0458
// CMD	         ON	      0000
//	              OFF	      0020
//    	     ALLOFF	      0080
//	     ALLLIGHTSOFF	      0084
//	           BRIGHT	      0088
//	    	ALLLIGHTSON	      0094
//	              DIM	      0098
//	     EXTENDEDCODE	
//	      HAILREQUEST	
//	          HAILACK	
//	       PRESETDIM1	
//	       PRESETDIM2	
//	     EXTENDEDDATA	
//	         STATUSON	
//	        STATUSOFF	
//	    STATUSREQUEST	

// fc_index -> cmd 18:XXXX -> wsHandler[] -> appQ -> fcc.dqMqttPubTask -> ecc.mqttQ -> ecc.dqMqttQ publish XXXX
// epp fcx:XXXX -> appEPext[] -> appQ -> fcc.dqMqttPubTask -> ecc.mqttQ -> ecc.dqMqttQ publish XXXX
// mqtt subscribe XXXX -> mqttHandler[] -> eccQ -> fcc.dqFCTask -> fcc.fc(XXXX)
//
//fc_index.html (x10 virtual keypad encodes 16 bits in ascii char format 0xXXXX) 
//  -> wsHandler(doc[cmd]:18 doc[fcx]:XXXX]) 
//    XXXX -> appQ 
//curl http://<fc_ip>/fcx/XXXX
//  -> appEndPointExtension(epp[cmd]:fcx epp[action]:XXXX]) 
//    XXXX -> appQ 
//appQ
//  -> fcc.dqMqttPubTask
//    appQ -> XXXX 
//    -> ecc.qMqttMsg(/firecracker/cmd, XXXX)
//      MqttMsg([topic]:/Firecracker/cmd [payload]:XXXX) -> mqttQ
//    -> ecc.dqMqttTask
//                        mqttQ -> publish([topic], [payload])
//mqtt subscribe /Firecracker/cmd 
//  -> mqttHandler(MqttMsg[topic], MqttMsg[payload]) 
//    M([topic],[payload] -> eccQ 
//  -> fcc.dqFirecrackerTask 
//    eccQ -> fcc.fc(MqttMsg[payload])

//EspCoreClass     ecc;
FirecrackerClass fcc;

// This function is used to create a delay for specified time
static inline void delay_time(uint32_t dt, char type = 'u')
{
    uint32_t total_us = (type == 'u') ? dt : dt * 1000;
    ets_delay_us(total_us);
}

uint16_t FirecrackerClass::str2hex(const String& hexstr) {
   uint16_t hex = strtol(hexstr.c_str(), NULL, 16);   
   return hex;
}

void FirecrackerClass::pwrFCoff() {
   // _FC_LO is defined as 1 which is intentionally inverted to drive 
   // transistors correctly to provide a LOW level to the FC pins
   gpio_set_level(fcc.fcp.dtr, _FC_LO);
   gpio_set_level(fcc.fcp.rts, _FC_LO);

#ifdef _DKV1
   // DevKitV1 powers the CM17A FC directly from vcc
   // so this power switch is used to turn on/off GND 
   gpio_set_level(fcc.fcp.pwr,_OFF);
#endif 
   fcc.fcp.fcOff = true;
   ecc._fdpl("fc powered off",_DBG3_MSG);
}

// emulate a Firecracker reset sequence
void FirecrackerClass::pwrFCon() {

   // Initial idle State = both pins high 
   // _FC_HI is defined as 0 which is intentionally inverted to drive 
   // transistors correctly to provide a HIGH level to the FC pins
   gpio_set_level(fcc.fcp.dtr,_FC_HI);
   gpio_set_level(fcc.fcp.rts,_FC_HI);    

#ifdef _DKV1
   // DevKitV1 powers the CM17A FC directly from vcc
   // so this power switch is used to turn on/off GND 
   gpio_set_level(fcc.fcp.pwr,_ON);
#endif 

   delay_time(fcc.fcp.strTm, 'm'); // Start Time
    
   fcc.pwrFCoff();
 
   // Low State = both pins low turns off the firecracker interface
   // _FC_LO is defined as 1 which is intentionally inverted to drive 
   // transistors correctly to provide a LOW level to the FC pins
   //gpio_set_level(fcc.fcp.dtr, _FC_LO);
   //gpio_set_level(fcc.fcp.rts, _FC_LO);

//#ifdef _DKV1
   // DevKitV1 powers the CM17A FC directly from vcc
   // so this power switch is used to turn on/off GND 
   //gpio_set_level(fcc.fcp.pwr,_OFF);
//#endif 

   delay_time(fcc.fcp.rstTm, 'm'); // Reset Time
        
   // _FC_HI is defined as 0 which is intentionally inverted to drive 
   // transistors correctly to provide a HIGH level to the FC pins
   gpio_set_level(fcc.fcp.dtr, _FC_HI);
   gpio_set_level(fcc.fcp.rts, _FC_HI);

#ifdef _DKV1
   // DevKitV1 powers the CM17A FC directly from vcc
   // so this power switch is used to turn on/off GND 
   gpio_set_level(fcc.fcp.pwr,_ON);
#endif 
    
   delay_time(fcc.fcp.strTm, 'm'); // Start Time
   //fcc.fcp.wkTm = _FC_WAKE_TIME;                                
   fcc.fcp.fcOff = false;
   ecc._fdpl("fc powered on",_DBG3_MSG);
}

/*
 * Initialize the GPIO pins for the FC interface
 * RTS and DTR are set as outputs, and the initial state is set to idle (both high).
 * both pins drive 2n3904 transistors to drive the FC interface. DTR and RTS are inverted to drive the transistors correctly.
 * The FC gets its power from the 2n3904 transistors and the 5V supply from the ESP32 board. The FC must have either DTR or RTS
 * high to power up the FC. The FC will not work if both DTR and RTS are low.
 */

void FirecrackerClass::initFCgpio() {
   ecc._fdpl("fc initFCgpio ...");

   // Note 1ULL is an unsigned long long represeting 1 as a 64 bit number
   //       => 0000 0000 0000 0000 - 0000 0000 0000 0001 
   gpio_config_t io = {
      .pin_bit_mask = (1ULL << fcc.fcp.rts) | (1ULL << fcc.fcp.dtr)
#ifdef _DKV1
                    | (1ULL << fcc.fcp.pwr)
#endif      
      ,
      .mode         = GPIO_MODE_OUTPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type    = GPIO_INTR_DISABLE
   };

   gpio_config(&io);

   fcc.pwrFCon();
}

// This function transmits the 40-bit bitstream to the FireCracker interface
// using 2 GPIO pins connected to rts and dtr of the serial port of the fc.
int FirecrackerClass::txBitStream(const uint8_t bits[40]) {
   ecc._fdpl("fc txBitStream ... ",_DBG2_MSG);

   // check for null pointer
   if (!bits) return -1;
   //LastLedAutoState = LedAutoState;
   //LedAutoState = _OFF;
   ecc.vars.bl.appBusy = true;
   ecc.ledAction(EspCoreClass::_LED_ON); 
   delay(1);
   /* 
    * iterate through the 40 bits and set the GPIO pins accordingly. A "1" is represented by 
    * setting DTR high and RTS low, while a "0" is represented by setting DTR low and RTS high. 
    * After each bit, there is a gap where both pins are set high.
   */
   int repeats        = fcc.fcp.frRpt;
   int fTm            = fcc.fcp.frmTm;
   int bTm            = fcc.fcp.bitTm;
   int gTm            = fcc.fcp.gapTm;
   gpio_num_t dtr     = fcc.fcp.dtr;
   gpio_num_t rts     = fcc.fcp.rts;

   for (int r = 0; r < repeats; r++) {
      for (int i = 0; i < 40; ++i) {
         if (bits[i]) {  // "1": DTR=0, RTS=1
            gpio_set_level(dtr, _FC_LO);
            // wait for the specified bit duration
            delay_time(bTm);
            gpio_set_level(dtr, _FC_HI);            
        
         } else { // "0": DTR=1, RTS=0
            gpio_set_level(rts, _FC_LO);
            // wait for the specified bit duration
            delay_time(bTm);
            gpio_set_level(rts, _FC_HI);            
         }
         // wait for the specified bit gap duration
         delay_time(gTm);
      }
      // wait for the specified repeat duration
      if (r < repeats) {
         delay_time(fTm,'m');
      }
   }
   delay(1);
   //ecc.setLed(_OFF);
   ecc.ledAction(EspCoreClass::_LED_OFF); 
   //LedAutoState = LastLedAutoState;
   ecc.vars.bl.appBusy = false;
   return 0;
}

/*
 * This function sends a command to the FireCracker interface. It formats the command
 * into a frame, builds a 40 bit bitstream from the frame, and transmits the bitstream 
 * using GPIO pins. The command is repeated a specified number of times with a delay 
 * between each transmission. After sending the command, it prints the frame 
 * and bitstream for debugging purposes.
*/ 
void FirecrackerClass::firecracker(uint16_t cmd16) {

   ecc._fdpl("fc Firecracker ... " + String(cmd16),_DBG1_MSG);

   if (fcc.fcp.wkTm <= 0 || fcc.fcp.fcOff) fcc.pwrFCon();
   while (fcc.fcp.fcOff) delay(1);

   uint16_t hdr16 = fcc.fcp.hdr; //0xD5AA;
   uint8_t  ftr8  = fcc.fcp.ftr; //0xAD;

   uint8_t frame[5];
   uint8_t bits[40];

   // format the frame with the header, command, and footer
   // fmtFCframe(hdr16, cmd16, ftr8, frame);
   frame[0] = hdr16 >> 8;
   frame[1] = hdr16 & 0xFF;
   frame[2] = cmd16 >> 8;
   frame[3] = cmd16 & 0xFF;
   frame[4] = ftr8;

   // build the 40 bit bitstream from the frame
   // buildFCbitstream(frame, bits);
   for (int i = 0; i < 5; ++i) {
      uint8_t b = frame[i];
      for (int bit = 7; bit >= 0; --bit) {
         int idx = i * 8 + (7 - bit);
         bits[idx] = (b >> bit) & 1;
      }  
   }
   ecc._fdpl("fc b4 send");
   // request transmission of the firecracker 40 bit bitstream  
   fcc.txBitStream(bits);
   ecc._fdpl("fc sent: ");
   // print the frame and bitstream for debugging purposes 
   //char buf[81];
   int pos = 0;
   for(int i = 0; i < 5; i++) {
      uint8_t b = frame[i];
      for(int bit = 7; bit >= 0; bit--) {
         bits[i*8 + (7-bit)] = (b >> bit) & 1;
         ecc._fdp((bits[i*8 + (7-bit)]) ? "1" : "0");
         if (bit == 4) ecc._fdp("-");
         if (bit == 0) ecc._fdpl("");
      }
   }

   fcc.fcp.wkTm = _FC_WAKE_TIME;

   //buf[pos] = 0;
   //ecc._fdpl("fc bitstream output\n" + String(buf),_INFO_MSG);
    
   // for ESP32 operation we are NOT turning off the FireCracker interface
   // after sending the command unlike in Windoz where the firecracker program
   // terminates and requires enough time for the firecracker rf to be sent 
   // before terminating the program
   // delay_time(stpTm,'m');
}

bool FirecrackerClass::wsHandler(JsonDocument& doc) {
   ecc._fdpl("fc wsHandler ...",_DBG1_MSG);
   //if (deserializeJson(doc, msg) != DeserializationError::Ok) return false;
   
   int cmd = doc["cmd"] | -1;

   if (cmd == 18) {
      char buf[_APPQ_BUF_MAXLEN];
      //const char *hex = doc["fcx"] | nullptr;
      //if (!hex) return;

      // publish to MQTT
      String hex = doc["fcx"];
      if (hex.length() != 4) return false;
      // sanitize: 4 hex chars
      hex.toLowerCase();
      hex.toCharArray(buf, _APPQ_BUF_MAXLEN);
   
      // uint16_t val = strtol(hex.c_str(), NULL, 16);
      // int16_t val = strtol((char*)fcx, NULL, 16);
      ecc._fdpl("fc wsHandler ... " + hex,_DBG2_MSG);
      ecc._fdpl("fc inc appQ " + ecc.qDepth(ecc._APP_Q,"inc"),_DBG3_MSG);
      xQueueSend(ecc.app2eccQ, buf, 0);
   }
   return true;
}

String FirecrackerClass::appEndPointExtension(const EspCoreClass::EndPoint& epp) {
   ecc._fdpl("fc appEndPointExtension ...",_DBG1_MSG);
   String rtnMsg = "Unknown_Request";
   if (epp.cmd == "fcx") {
      String hex = epp.action;

      // sanitize: 4 hex chars
      hex.toLowerCase();
      if (hex.length() == 4) {
         char buf[_APPQ_BUF_MAXLEN];
         hex.toCharArray(buf, _APPQ_BUF_MAXLEN);
         ecc._fdpl("fc addEndPointExtension hex ... " + hex ,_DBG2_MSG);
         ecc._fdpl("fc inc appQ " + ecc.qDepth(ecc._APP_Q,"inc"),_DBG3_MSG);
         xQueueSend(ecc.app2eccQ, buf, 0);
         rtnMsg = epp.cmd + " " + hex;
      }
   }   
   return rtnMsg;
}

void FirecrackerClass::dqMqttPubTask(void *pv) {
   ecc._fdpl("fc dqMqttPubTask ... ",_DBG1_MSG);
   char buf[_APPQ_BUF_MAXLEN];
   for (;;) {
      if (xQueueReceive(ecc.app2eccQ, buf, portMAX_DELAY) == pdTRUE) {
         String cmd(buf);
         ecc._fdpl("fc dqMqttPubTask ... " + cmd ,_DBG2_MSG);
         ecc._fdpl("fc dec appQ " + ecc.qDepth(ecc._APP_Q,"dec"),_DBG3_MSG);
         ecc.qMqttMsg(_APP_TOPIC, cmd);        
      }
   }
}

// subscriber link
bool FirecrackerClass::mqttHandler(const EspCoreClass::MqttMsg& M) {
   ecc._fdpl("fc mqttHandler ... " + String(M.topic) + " " + String(M.payload) ,_DBG1_MSG);

   //if (M.topic == _APP_TOPIC) {
   if (strcmp(M.topic, _APP_TOPIC) == 0) {
      char buf[_APPQ_BUF_MAXLEN];
      strncpy(buf, M.payload, _APPQ_BUF_MAXLEN - 1);
      buf[_APPQ_BUF_MAXLEN - 1] = '\0';
      ecc._fdpl("fc nqttHandler handle payload",_DBG2_MSG);
      // push ASCII hex payload into ecc2appQ
      ecc._fdpl("fc inc eccQ " + ecc.qDepth(ecc._ECC_Q,"inc"),_DBG3_MSG);
      xQueueSend(ecc.ecc2appQ, buf, 0);
      return true;
   }
   return false;
}

void FirecrackerClass::dqFirecrackerTask(void *pv) {
   ecc._fdpl("fc dqFirecrackerTask ... " ,_DBG1_MSG);
   char buf[_APPQ_BUF_MAXLEN];
   uint16_t cmd16;
   for (;;) {
      
      if (xQueueReceive(ecc.ecc2appQ, buf, portMAX_DELAY) == pdTRUE) {
         String cmd(buf);
         ecc._fdpl("fc dqFirecrackerTask ... " + cmd ,_DBG2_MSG);
         ecc._fdpl("fc dec eccQ "  + ecc.qDepth(ecc._ECC_Q,"dec"),_DBG3_MSG);
         cmd16 = fcc.str2hex(cmd);
         fcc.firecracker(cmd16);
      }
   }
}

void FirecrackerClass::begin() {
   ecc.appWsHandler = [&](JsonDocument& doc){
      return fcc.wsHandler(doc);
   };

   ecc.appEndPointExtension = [&](const EspCoreClass::EndPoint& epp){
      return fcc.appEndPointExtension(epp);
   };

   ecc.appMqttHandler = [&](const EspCoreClass::MqttMsg& M){
      return fcc.mqttHandler(M);
   };

   //ecc.setAppMqttTopic(_APP_TOPIC);
   ecc.setAppHtml(_APP_HTML);
 
   fcc.initFCgpio();

   xTaskCreate(
      FirecrackerClass::dqFirecrackerTask,
      "FC_TX",
      4096,
      nullptr,
      3,
      nullptr
   );

   xTaskCreate(
      FirecrackerClass::dqMqttPubTask,
      "FC_MQTT",
      4096,
      nullptr,
      2,
      nullptr
   );

}

void FirecrackerClass::loop() {  
   if (ecc.vars.enm.mqttState == ecc._MQTT_RESUB) {
      ecc._fdpl("fc reconnect Topics ... mqtt ... " + String(_APP_TOPIC) ,_DBG3_MSG);

      ecc.mqtt.subscribe(_APP_TOPIC);
      ecc.vars.enm.mqttState = ecc._MQTT_CON;
      ecc._fdpl("fc MQTT NOW FULLY CONNECTED ",_DBG3_MSG);
   }

   if (fcc.fcp.wkTm > 0) {
      fcc.fcp.wkTm--;
      if (fcc.fcp.wkTm == 0) fcc.pwrFCoff();
   }

   delay(100);
}
