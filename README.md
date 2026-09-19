Been using X10 continuously since the early 80's and so this project was to expand the tech to smart phones and HA via MQTT and ESP32.
A slight mod has been made to a CM17A (schematic in root folder) to interface to an Esp32 ... this Sketch requires my ESP Core Lib 
ESP Core Lib provides the MQTT links to allow Home Assistant (HA) to request sending an RF command to the X10 RF Receiver 

This has been updated ... CM17A generic software turns off the power to the internal MCU (pic 12C508A) after doing its rf transmission 
... my original esp32 version turned on the power and left it on ... my reason for doing this was simply thinking it would speed up the 
transmission requests ... the issue became the CM17A MCU somehow locked up, refusing to do any further transmission after several hours
of being powered

This revised version has a timer that starts on any transmission and retains power for 30 seconds of no activity then turns off the power
so basically if your automation (HA or other) needs to send several x10 commands the first command could take a few hundred mSecs but 
subsequent commands not have to go through the reset power delay ... further in case the CM17A MCU does lock up there is a new endpoint
rstfc which will go through the generic software power reset dance 

This project provides 3 methods of triggering X10 RF commands via the CM17A
1) endpoint <ip>/fcx/XXXX
2) ESP Web page that simulates an HR12A hand held 8/16 x10 rf controller
3) mqtt topic /Firecracker/cmd  payload XXXX (HA friendly)
