/*
 https://en.wikipedia.org/wiki/Svara
 
 The absolute frequencies for all svaras are variable, and are determined relative to the saptak or octave. E.g. given Sa 240 Hz, Re 270 Hz, Ga 288 Hz, Ma 320 Hz, Pa 360 Hz, Dha 405 Hz, and Ni 432 Hz, then the Sa after the Ni of 432 Hz has a frequency of 480 Hz i.e. double that of the lower octave Sa, and similarly all the other 6 svaras. Considering the Sa of the Madhya Saptak then frequencies of the other svaras will be,

                  Sa      Re       Ga     Ma      Pa      Dha     Ni
  Mandra Saptak: 120 Hz, 135 Hz, 144 Hz, 160 Hz, 180 Hz, 202.5 Hz, 216 Hz.}
  Madhya Saptak: 240 Hz, 270 Hz, 288 Hz, 320 Hz, 360 Hz, 405 Hz, 432 Hz.} 
  Taara Saptak: 480 Hz, 540 Hz, 576 Hz, 640 Hz, 720 Hz, 810 Hz, 864 Hz.}
   
*/

#include <esp_now.h>
#include <WiFi.h>

#define lineIn ADC1_CHANNEL_0          // ADC 1 channel 0 GPIO36 for analog microphone connecttion
#define lineOut 25                     // DAC pin for headphone or amplified speaker connection
#define PTT_PIN 2                      // PTT Push to transmitt switch connection
#define touchThreshold 50              // TTT (Touch To Transmit) threshold
#define audioBytes  240                // ESPNow allow upto 250 bytes packets

uint8_t audioData[audioBytes] = 
   
   // Sin wave
   {
    0x80, 0x83, 0x87, 0x8A, 0x8E, 0x91, 0x95, 0x98, 0x9B, 0x9E, 0xA2, 0xA5, 0xA7, 0xAA, 0xAD, 0xAF,
    0xB2, 0xB4, 0xB6, 0xB8, 0xB9, 0xBB, 0xBC, 0xBD, 0xBE, 0xBF, 0xBF, 0xBF, 0xC0, 0xBF, 0xBF, 0xBF,
    0xBE, 0xBD, 0xBC, 0xBB, 0xB9, 0xB8, 0xB6, 0xB4, 0xB2, 0xAF, 0xAD, 0xAA, 0xA7, 0xA5, 0xA2, 0x9E,
    0x9B, 0x98, 0x95, 0x91, 0x8E, 0x8A, 0x87, 0x83, 0x80, 0x7C, 0x78, 0x75, 0x71, 0x6E, 0x6A, 0x67,
    0x64, 0x61, 0x5D, 0x5A, 0x58, 0x55, 0x52, 0x50, 0x4D, 0x4B, 0x49, 0x47, 0x46, 0x44, 0x43, 0x42,
    0x41, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x41, 0x42, 0x43, 0x44, 0x46, 0x47, 0x49, 0x4B,
    0x4D, 0x50, 0x52, 0x55, 0x58, 0x5A, 0x5D, 0x61, 0x64, 0x67, 0x6A, 0x6E, 0x71, 0x75, 0x78, 0x7C
   };
   

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  Serial.print("bytes received: ");
  Serial.print(len);
  uint8_t dataIn;
  for (int i=0;i<len;i++) {
    dataIn += (uint8_t)data[i];
  dacWrite(lineOut, dataIn);
  Serial.print("Data in: ");
  Serial.println(dataIn);
 }
}

void setup() {
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

 esp_now_register_recv_cb(OnDataRecv);
 
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
}
 
void loop() {
  
  if ( /*digitalRead(PTT_PIN) == LOW || */touchRead(4) < touchThreshold )
  {  
  Serial.print("Touch Value 4: ");
  Serial.println(touchRead(4));
  int i = 0;
  // Send audio packet via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *) audioData, sizeof(audioData)); 
  
  if (result == ESP_OK) {
    Serial.println("Sending audio successfully...");
  }
  else {
    Serial.println("Error sending the audio data...");
  }
  
  dacWrite(lineOut, audioData[i]);           // Play audio locally via DAC.
  if (i >= audioBytes) i = 0; 
  
  }
}
