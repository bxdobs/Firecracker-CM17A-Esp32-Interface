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
//  X10 Firecracker    RF CODE
//  ZONE            A       6000
//                  B       7000
//                  C       4000
//                  D       5000
//                  E       8000
//                  F       9000
//                  G       A000
//                  H       B000
//                  I       E000
//                  J       F000
//                  K       C000
//                  L       D000
//                  M       0000
//                  N       1000
//                  O       2000
//                  P       3000
// UNIT          01       0000
//                 02       0010
//                 03       0008
//                 04       0018
//                05        0040
//                 06       0050
//                 07       0048
//                 08       0058
//                 09       0400
//                 10       0410
//                 11       0408
//                 12       0418
//              13        0440
//                 14       0450
//                 15       0448
//                 16       0458
// CMD           ON       0000
//                OFF       0020
//           ALLOFF       0080
//       ALLLIGHTSOFF       0084
//             BRIGHT       0088
//        ALLLIGHTSON       0094
//                DIM       0098
//       EXTENDEDCODE 
//        HAILREQUEST 
//            HAILACK 
//         PRESETDIM1 
//         PRESETDIM2 
//       EXTENDEDDATA 
//           STATUSON 
//          STATUSOFF 
//      STATUSREQUEST 

// this sketch integrates with esp core 
//    there is a web page in littleFS fc_index.html
//                                    action in this web page send a web socket command 18:XXXX
// Webpage process pushes to mqqtt
// fc_index.html -> cmd 18:XXXX -> wsHandler[] -> appQ -> fcc.dqMqttPubTask -> ecc.mqttQ -> ecc.dqMqttQ publish XXXX
//
// Endpoint process pushes to mqtt
// epp fcx:XXXX -> appEPext[] -> appQ -> fcc.dqMqttPubTask -> ecc.mqttQ -> ecc.dqMqttQ publish XXXX
//
// mqtt subscribe runs fcc.firecracker()
// mqtt subscribe XXXX -> mqttHandler[] -> eccQ -> fcc.dqFCTask -> fcc.firecracker(XXXX)
//
//fc_index.html (x10 virtual keypad encodes 16 bits in ascii char format 0xXXXX) 
//  -> wsHandler(doc[cmd]:18 doc[fcx]:XXXX]) 
//    XXXX -> appQ 
//
//curl http://<fc_ip>/fcx/XXXX
//  -> EndPointExtension(epp[cmd]:fcx epp[action]:XXXX]) 
//    XXXX -> appQ
// 
//appQ
//  -> fcc.dqMqttPubTask
//    appQ -> XXXX 
//    -> ecc.qMqttMsg(/firecracker/cmd, XXXX)
//      MqttMsg([topic]:/Firecracker/cmd [payload]:XXXX) -> mqttQ
//    -> ecc.dqMqttTask
//                        mqttQ -> publish([topic], [payload])
//
//mqtt subscribe /Firecracker/cmd 
//  -> mqttHandler(MqttMsg[topic], MqttMsg[payload]) 
//    M([topic],[payload] -> eccQ 
//  -> fcc.dqFirecrackerTask 
//    eccQ -> fcc.fc(MqttMsg[payload])

//EspCoreClass     ecc;
FirecrackerClass fcc;

// This function is used to create accurate uninterruptable delay times
static inline void delay_time(uint32_t dt, char type = 'u')
{
    uint32_t total_us = (type == 'u') ? dt : dt * 1000;
    ets_delay_us(total_us);
}

// convert string to hex
uint16_t FirecrackerClass::str2hex(const String& hexstr) {
   uint16_t hex = strtol(hexstr.c_str(), NULL, 16);   
   return hex;
}
const char * const FirecrackerClass::zones[] = {
      "M", "N", "O", "P",
      "C", "D", "A", "B",
      "E", "F", "G", "H",
      "K", "L", "I", "J"
};

const uint16_t FirecrackerClass::unitCodes[] = {
      0x0458,
      0x000, 0x008, 0x010, 0x018,
      0x040, 0x048, 0x050, 0x058,
      0x400, 0x408, 0x410, 0x418,
      0x440, 0x448, 0x450, 0x458
};

const char * const FirecrackerClass::units[] = {
      "Unknown",
      "01", "03", "02", "04",
      "05", "07", "06", "08",
      "09", "11", "10", "12",
      "13", "15", "14", "16"
};

const uint8_t FirecrackerClass::cmdCodes[] = {
      0x9C, 0x80, 0x84, 0x88, 0x94, 0x98
};

const char * const FirecrackerClass::cmds[] = {
      "Unknown",
      "ALLOFF",
      "ALLIGHTSOFF",
      "BRIGHT",
      "ALLIGHTSON",
      "DIM"
}; 

// pwrFC on off OR 
bool FirecrackerClass::pwrFC(int pwrState = -2) {
   if (pwrState == _FC_ON) {
      // Initial idle State = both pins high 
      // _FC_HI is defined as 0 which is intentionally inverted to drive 
      // transistors correctly to provide a HIGH level to the FC pins
      gpio_set_level(fcc.fcp.dtr,_FC_HI);
      gpio_set_level(fcc.fcp.rts,_FC_HI);    

#ifdef _DKV1
      // DevKitV1 powers the CM17A FC directly from vcc
      // so this power switch is used to turn on/off GND 
      gpio_set_level(fcc.fcp.pwr,_FC_ON);
#endif 
      // set on state flag
      fcc.fcp.fcOn = true;
   } else if (pwrState == _FC_OFF) {
        // _FC_HI is defined as 0 which is intentionally inverted to drive 
      // transistors correctly to provide a HIGH level to the FC pins
      gpio_set_level(fcc.fcp.dtr, _FC_LO);
      gpio_set_level(fcc.fcp.rts, _FC_LO);

#ifdef _DKV1
      // DevKitV1 powers the CM17A FC directly from vcc
      // so this power switch is used to turn on/off GND 
      gpio_set_level(fcc.fcp.pwr,_FC_OFF);
#endif 
      // clear on state flag
      fcc.fcp.fcOn = false;
   }  
   // return current on state value
   return fcc.fcp.fcOn;
}

// need an endpoint to turn on or off or get status of power and time
void FirecrackerClass::resetFC(void) {
   ecc._fdpl("fc reset ...");
  
   // apply power to the Firecracker
   fcc.pwrFC(_FC_ON);
   
   // allow the Firecracker MCU Power to settle
   delay_time(fcc.fcp.strTm, 'm'); // Start Time
   
   // turn off power to the Firecracker
   fcc.pwrFC(_FC_OFF);
 
   // allow residual capacitance to disipate
   delay_time(fcc.fcp.rstTm, 'm'); // Reset Time
   
   // reapply power to the Firecracker
   fcc.pwrFC(_FC_ON);
   
   // allow the Firecracker MCU power to settle
   delay_time(fcc.fcp.strTm, 'm'); // Start Time
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
   //       the bit is shifted the number of bits to the left defined by the 
   //       gpio value rts, dtr, pwr, etc
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

   // based on the config io params these pins are set to output with no internal resistors
   gpio_config(&io);

}

// This function transmits the 40-bit bitstream to the FireCracker interface
// using 2 GPIO pins connected to rts and dtr of the serial port of the fc.
int FirecrackerClass::txBitStream(const uint8_t bits[40]) {
   ecc._fdpl("fc txBitStream",_DBG1_MSG);

   // check for null pointer
   if (!bits) return -1;

   // ensure that active logging halts while streaming the 40 fcx bits
   ecc.vars.bl.appBusy = true;

   // turn on system/board led to indicate 40 bit stream is being sent
   ecc.ledAction(EspCoreClass::_LED_ON); 
   
   // add a tiny 1mS delay
   delay(1);
   /* 
    * iterate through the 40 bits and set the GPIO pins accordingly. A "1" is represented by 
    * setting DTR high and RTS low, while a "0" is represented by setting DTR low and RTS high. 
    * After each bit, there is a gap where both pins are set high.
   */

   // make a local copy of active vars to ensure the code doesn't waste time refetching vars
   int repeats        = fcc.fcp.frRpt;
   int fTm            = fcc.fcp.frmTm;
   int bTm            = fcc.fcp.bitTm;
   int gTm            = fcc.fcp.gapTm;
   gpio_num_t dtr     = fcc.fcp.dtr;
   gpio_num_t rts     = fcc.fcp.rts;

   for (int r = 0; r < repeats; r++) {
      for (int i = 0; i < 40; ++i) {
         // handle 1
         if (bits[i]) {  // "1": DTR=0, RTS=1
            gpio_set_level(dtr, _FC_LO);
            // wait for the specified bit duration
            delay_time(bTm);
            gpio_set_level(dtr, _FC_HI);            
        
         // handle 0
         } else { // "0": DTR=1, RTS=0
            gpio_set_level(rts, _FC_LO);
            // wait for the specified bit duration
            delay_time(bTm); // bit time
            gpio_set_level(rts, _FC_HI);            
         }

         // wait for the specified bit gap duration
         delay_time(gTm); // gap time
      }

      // wait for the specified repeat duration
      if (r < repeats) {
         delay_time(fTm,'m'); // frame time
      }
   }
   // add another tiny 1mS delay
   delay(1);
 
   // turn off System/board led indicating fcx frame was completely sent
   ecc.ledAction(EspCoreClass::_LED_OFF); 

   // allow logging to resume 
   ecc.vars.bl.appBusy = false;

   return 0;
}

// This function decodes 16 bit fcx codes to X10 codes
String FirecrackerClass::dcfc(uint16_t code) {
   String decode = fcc.zones[(code >> 12) & 0x0F];
   uint16_t encode = code & 0xF000;

   // idx 0 represents an unknown command
   int idx = 0;

   // bit 7 == 1 => Command-only code
   if ((code & 0x0080) != 0) {

      // mask off the command code
      uint8_t cmdCode = code & fcc.cmdCodes[0];
      encode += cmdCode;

      // attempt to match a command code
      for (int i = 1; i <= 5; i++) {
         if (cmdCode == fcc.cmdCodes[i]) {
            idx = i;
            break;
         }
      }

      decode += " ";
      decode += fcc.cmds[idx];

   } else { // Unit + On | Off

      // Unit code
      uint16_t unitCode = code & fcc.unitCodes[0];
      encode += unitCode;

      for (int i = 1; i <= 16 ; i++) {
         if (unitCode == fcc.unitCodes[i]) {
            idx = i;
            break;
         }
      }

      decode += " ";
      decode += fcc.units[idx];
      decode += ((code & 0x0020) == 0) ? " On" : " Off";
      encode += ((code & 0x0020) == 0) ? 0x0000 : 0x0020;
   }

   return decode + ((encode == code) ? "" : " Err:" + String(code, HEX) + " " + String(encode, HEX));
}  

/*
 * This function sends a command to the FireCracker interface. It formats the command
 * into a frame, builds a 40 bit bitstream from the frame, and transmits the bitstream 
 * using GPIO pins. The command is repeated a specified number of times with a delay 
 * between each transmission. After sending the command, it prints the frame 
 * and bitstream for debugging purposes.
*/ 
void FirecrackerClass::firecracker(uint16_t cmd16) {

   ecc._fdpl("fc Firecracker ... " + String(cmd16) + " " + String(fcc.dcfc(cmd16)));

   // potential failure point? ... we turn on power then wait forever for a power on signal
   if (fcc.fcp.wkTm <= 0 || !fcc.fcp.fcOn) fcc.pwrFC(_FC_ON);

   // ensure we aren't hitting this without an intercommand delay
   delay_time(fcc.fcp.strTm, 'm'); // Start Time
   
   //while (!fcc.fcp.fcOn) delay(1);

   uint16_t hdr16 = fcc.fcp.hdr; // fcx header 0xD5AA;
   uint8_t  ftr8  = fcc.fcp.ftr; // fcx footer 0xAD;

   uint8_t frame[5];             // 5 byte frame to hold ascii hex (5 x 8 => 40 binary bits)
   uint8_t bits[40];             // 40 bit frame to hold binary value

   // format the frame with the header, command, and footer
   // fmtFCframe(hdr16, cmd16, ftr8, frame);
   frame[0] = hdr16 >> 8;
   frame[1] = hdr16 & 0xFF;
   frame[2] = cmd16 >> 8;
   frame[3] = cmd16 & 0xFF;
   frame[4] = ftr8;

   // build the 40 bit bitstream from the frame
   for (int i = 0; i < 5; ++i) {
      uint8_t b = frame[i];
      for (int bit = 7; bit >= 0; --bit) {
         int idx = i * 8 + (7 - bit);
         bits[idx] = (b >> bit) & 1;
      }  
   }

   // transmit the 40 bit firecracker frame  
   fcc.txBitStream(bits);
   
   // print the cmd16, decoded val + frame for debugging purposes
   ecc._fdpl("FC firecracker " + dcfc(cmd16)) ;

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

   // ensure we aren't hitting this without an intercommand delay
   delay_time(fcc.fcp.stpTm, 'm'); // Stop Time 
   
   // request the wake timer to be set to keep power applied for wake time
   fcc.fcp.wkTm = _FC_WAKE_TIME; 
}

// this handler is called when the FC web page requests an fcx cmd
// all fcx cmds are published to the mqtt broker via the app queue   
bool FirecrackerClass::wsHandler(JsonDocument& doc) {
   ecc._fdpl("fc wsHandler event", _DBG1_MSG);
   
   // get the web cmd from the json package 
   // | -1 sets cmd default value if json package is empty {}
   int cmd = doc["cmd"] | -1;

   // the web page prefixes fcx commands with cmd 18 
   if (cmd == 18) {
      // require a capture buffer for cmd content
      char buf[_APPQ_BUF_MAXLEN];

      // get the 16 bit cmd code from the fcx command
      String hex = doc["fcx"];
      if (hex.length() != 4) return false;

      // force 4 hex chars to lower case
      hex.toLowerCase();
      hex.toCharArray(buf, _APPQ_BUF_MAXLEN);
   
      ecc._fdpl("fc wsHandler 18_fcx: " + hex,_DBG2_MSG);
      
      // increment the application Queue Counter
      ecc._fdpl("fc wsHandler inc appQ " + ecc.qDepth(ecc._APP_Q,"inc"),_DBG3_MSG);
   
      // as fcx cmds can come from the web page or from endpoints we push
      // them in to an app queue so they are processed out to the MQTT broker in the order rcvd
      xQueueSend(ecc.app2eccQ, buf, 0);
   }
   return true;
}

String FirecrackerClass::EndPointExtension(const EspCoreClass::EndPoint& epp) {
   ecc._fdpl("fc EndPointExtension ...",_DBG1_MSG);
   String rtnMsg = "Unknown_Request";
   if (epp.cmd == "fcx") {
      String hex = epp.action;

      // fcx requires 4 ascii hex chars
      hex.toLowerCase();
      if (hex.length() == 4) {
         char buf[_APPQ_BUF_MAXLEN];
         hex.toCharArray(buf, _APPQ_BUF_MAXLEN);

         // debug message
         ecc._fdpl("fc EndPointExtension hex " + hex + " " + fcc.dcfc(fcc.str2hex(hex)) ,_DBG2_MSG);

         // increment the App Q for endpoint fcx commands
         ecc._fdpl("fc EndPointExtension inc appQ " + ecc.qDepth(ecc._APP_Q,"inc"),_DBG3_MSG);
         
         // add fcx cmd to app que
         xQueueSend(ecc.app2eccQ, buf, 0);
         rtnMsg = epp.cmd + " " + hex + " " + fcc.dcfc(fcc.str2hex(hex));
      }
   } else if (epp.cmd == "resetfc") {
      fcc.resetFC();
      rtnMsg = "Reset FC";
   }  else if (epp.cmd == "fcpwr") {
      ecc._fdpl("fc endpoint fcpwr " + epp.action);
      bool isFCon = fcc.fcp.fcOn;
      if (epp.action == "on") {
         isFCon = fcc.pwrFC(_FC_ON);
      } else if (epp.action == "off") {
         isFCon = fcc.pwrFC(_FC_OFF);
      }   
      rtnMsg = epp.cmd + ((isFCon) ? " on " : " off ") + String(fcc.fcp.wkTm);
   }
   return rtnMsg;
}

// dq values to publish to the mqtt broker
// this is a task loop that continuously runs to deque any pending mqtt publish topics
void FirecrackerClass::dqMqttPubTask(void *pv) {
   ecc._fdpl("fc dqMqttPubTask ... ");
   char buf[_APPQ_BUF_MAXLEN];
   for (;;) {
      // if there are items in the app queue then pull them
      if (xQueueReceive(ecc.app2eccQ, buf, portMAX_DELAY) == pdTRUE) {
         String cmd(buf);
         ecc._fdpl("fc dqMqttPubTask " + cmd ,_DBG2_MSG);

         // decrement app q counter
         ecc._fdpl("fc dqMqttPubTask dec appQ " + ecc.qDepth(ecc._APP_Q,"dec"),_DBG3_MSG);

         // q mqtt msg
         ecc.qMqttMsg(_APP_TOPIC, cmd);        
      }
   }
}

// subscriber link
bool FirecrackerClass::mqttHandler(const EspCoreClass::MqttMsg& M) {
   ecc._fdpl("fc mqttHandler ... " + String(M.topic) + " " + String(M.payload) ,_DBG1_MSG);

   if (strcmp(M.topic, _APP_TOPIC) == 0) {
      char buf[_APPQ_BUF_MAXLEN];
      strncpy(buf, M.payload, _APPQ_BUF_MAXLEN - 1);
      buf[_APPQ_BUF_MAXLEN - 1] = '\0';
      ecc._fdpl("fc nqttHandler handle payload",_DBG2_MSG);
      // push ASCII hex payload into ecc2appQ

      // increment ecc q depth
      ecc._fdpl("fc mqttHandler inc eccQ " + ecc.qDepth(ecc._ECC_Q,"inc"),_DBG3_MSG);
       
      // q ecc 
      xQueueSend(ecc.ecc2appQ, buf, 0);
      return true;
   }
   return false;
}

void FirecrackerClass::dqFirecrackerTask(void *pv) {
   ecc._fdpl("fc dqFirecrackerTask ... ");
   char buf[_APPQ_BUF_MAXLEN];
   uint16_t cmd16;
   for (;;) {
      
      if (xQueueReceive(ecc.ecc2appQ, buf, portMAX_DELAY) == pdTRUE) {
         String cmd(buf);
         ecc._fdpl("fc dqFirecrackerTask " + cmd ,_DBG2_MSG);

         ecc._fdpl("fc dqFirecrackerTask dec eccQ "  + ecc.qDepth(ecc._ECC_Q,"dec"),_DBG3_MSG);
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
      return fcc.EndPointExtension(epp);
   };

   ecc.appMqttHandler = [&](const EspCoreClass::MqttMsg& M){
      return fcc.mqttHandler(M);
   };

   //ecc.setAppMqttTopic(_APP_TOPIC);
   ecc.setAppHtml(_APP_HTML);
 
   fcc.initFCgpio();

   fcc.resetFC();
   
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
      if (!ecc.vars.bl.appBusy) fcc.fcp.wkTm--;

      if (fcc.fcp.wkTm == 0) {
         fcc.pwrFC(_FC_OFF);
      }   
   }

   delay(100);
}
