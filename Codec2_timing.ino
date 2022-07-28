// https://github.com/deulis/ESP32_Codec2/issues/4

#include <codec2.h>
#define lineOut 25                      // DAC pin (GPIO 25 or 26 of ESP32)

struct CODEC2* codec2_state;

int16_t sine1KHz[8] = { -21210 , -30000, -21210, 0 , 21210 , 30000 , 21210, 0 };
uint8_t rx_raw_audio_value = 0;
int16_t audioBuf[320];
uint8_t c2Buf[8];
boolean flagEncodeAudioBuf = false;
boolean flagDecodeC2Buf = false;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Started Audio Transport");

  // Set bitrate
  codec2_state = codec2_create(CODEC2_MODE_1600);
  // Set some tuning parameters
  codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);
}

void loop() {

  xTaskCreate(&codec2_watcher, "codec2_watcher_task", 30000, NULL, 5, NULL);

  // Encode a (fixed size) input buffer of type int16_t to a (fixed size) output buffer
  // CODEC2_MODE_1600 encodes 320 speech samples (320 * 2 = 640 bytes / 40ms of speech) into 8 bytes (64 bits)
  // Fill buffer with a sine wave
  for (int i = 0; i < 320; i++) audioBuf[i] = sine1KHz[i % 8];
  
  for (int i = 0; i < 320; i++) Serial.print(String(audioBuf[i], DEC) + " ");
  Serial.println();
  
  int startTimeEncode = millis();
  c2_encode();
  Serial.println("Done encoding, took ms: " + String(millis() - startTimeEncode));

  //for (int i = 0; i < 8; i++) Serial.print(String(c2Buf[i], DEC) + " ");Serial.println();
  
  int startTimeDecode = millis();
  c2_decode();
  Serial.println("Done decoding, took ms: " + String(millis() - startTimeDecode));

  //for (int i = 0; i < 320; i++) Serial.print(String(audioBuf[i], DEC) + " ");Serial.println();
  
  
  dacWrite(lineOut, (audioBuf, DEC));   // Play audio on DAC pin
  delay(5000);
} //End of loop

void c2_encode() {
  flagEncodeAudioBuf = true;
  while (flagEncodeAudioBuf) delay(1); // Wait for the codec2_watch
}

void c2_decode() {
  flagDecodeC2Buf = true;
  while (flagDecodeC2Buf) delay(1); // Wait for the codec2_watch
}

void codec2_watcher(void* parameter) {
  while (true) {
    // yield() DOES NOT work, that trips the WDT every 5 secs
    
    delay(1); // delay(1) is VITAL
    if (flagEncodeAudioBuf) { // Encode audio to Codec2 format for transport
      codec2_encode(codec2_state, c2Buf, audioBuf);
      flagEncodeAudioBuf = false; // Notify encode()
    }
    if (flagDecodeC2Buf) { // Decode Codec2 to Audio format
      codec2_decode(codec2_state, audioBuf, c2Buf);
      flagDecodeC2Buf = false; // Notify decode()
    }
  }
}
