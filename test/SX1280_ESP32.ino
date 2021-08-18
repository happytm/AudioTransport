/*
   RadioLib SX128x Transmit Example
   This example transmits packets using SX1280 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)
   Other modules from SX128x family can also be used.
   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx128x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

// Ebyte E28-2G4M27S (SX1280) module has the following connections to Wemos Lolin32:

 NSS pin:   5              Miso: 19
 DIO1 pin:  27             Mosi: 23
 NRST pin:  14             SCK:  18
 BUSY pin:  2              NSS: 5
 TX_EN:     16             RX_EN: 17

*/

// include the library
#include <RadioLib.h>

SX1280 radio = new Module(5, 27, 14, 2);

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // initialize SX1280 with default settings
  Serial.print(F("[SX1280] Initializing ... "));
  // carrier frequency:           2400.0 MHz
  // bandwidth:                   812.5 kHz
  // spreading factor:            9
  // coding rate:                 7
  // output power:                10 dBm
  // preamble length:             12 symbols
  // CRC:                         enabled
  int state = radio.begin(int state = radio.begin(2400.0, 812.5, 9, 7, 10, 12);
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // some modules have an external RF switch
  // controlled via two pins (RX enable, TX enable)
  // to enable automatic control of the switch,
  // call the following method
  
    radio.setRfSwitchPins(16, 17);
}

void loop() {
  /////////////////////////////////Trasmit packets///////////////////////////////////// 
  
  Serial.print(F("[SX1280] Transmitting packet ... "));

  // you can transmit C-string or Arduino string up to
  // 256 characters long
  // NOTE: transmit() is a blocking method!
  //       See example SX128x_Transmit_Interrupt for details
  //       on non-blocking transmission method.
  int state = radio.transmit("Hello World!");

  // you can also transmit byte array up to 256 bytes long
  /*
    byte byteArr[] = {0x01, 0x23, 0x45, 0x56, 0x78, 0xAB, 0xCD, 0xEF};
    int state = radio.transmit(byteArr, 8);
  */

  if (state == ERR_NONE) {
    // the packet was successfully transmitted
    Serial.println(F("success!"));
    
  } else if (state == ERR_PACKET_TOO_LONG) {
    // the supplied packet was longer than 256 bytes
    Serial.println(F("too long!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
/////////////////////////////////Receive packets/////////////////////////////////////
  
    Serial.print(F("[SX1280] Waiting for incoming transmission ... "));

  // you can receive data as an Arduino String
  // NOTE: receive() is a blocking method!
  //       See example ReceiveInterrupt for details
  //       on non-blocking reception method.
  String str;
  state = radio.receive(str);

  // you can also receive data as byte array
  /*
    byte byteArr[8];
    int state = radio.receive(byteArr, 8);
  */

  if (state == ERR_NONE) {
    // packet was successfully received
    Serial.println(F("success!"));

    // print the data of the packet
    Serial.print(F("[SX1280] Data:\t\t"));
    Serial.println(str);

    // print the RSSI (Received Signal Strength Indicator)
    // of the last received packet
    Serial.print(F("[SX1280] RSSI:\t\t"));
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    // print the SNR (Signal-to-Noise Ratio)
    // of the last received packet
    Serial.print(F("[SX1280] SNR:\t\t"));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

  } else if (state == ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);

  }
  // wait for a second before transmitting again
  delay(1000);
}
