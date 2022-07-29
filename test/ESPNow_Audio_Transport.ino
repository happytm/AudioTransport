#include <Arduino.h>
#include <driver/adc.h>
#include <codec2.h>                     // Thanks to https://github.com/deulis/ESP32_Codec2 for porting codec2 to arduino
#include <ButterworthFilter.h>          // In the codec2 folder of arduino library folder
#include <FastAudioFIFO.h>              // In the codec2 folder of arduino library folder
#include <esp_now.h>
#include <WiFi.h>

#define ADC_PIN ADC1_CHANNEL_0          // ADC 1 channel 0 (GPIO36 of ESP32)
#define lineOut 25                      // DAC pin (GPIO 25 or 26 of ESP32)
#define PTT_PIN 2

#define ADC_BUFFER_SIZE 320             // 40ms of voice in 8KHz sampling frequency
#define ENCODE_FRAME_SIZE 44            // 44 =  First four bytes non-audio + 40 bytes audio
#define ENCODE_CODEC2_FRAME_SIZE 8      // 8 bytes per packets x 5 packets = 40 bytes of audio packet
#define ENCODE_FRAME_HEADER_SIZE 4      // 4 bytes of non-audio data

int16_t adc_buffer[ADC_BUFFER_SIZE];                   // 320 = 40ms of voice in 8KHz sampling frequency
int16_t speech[ADC_BUFFER_SIZE];                       
int16_t output_buffer[ADC_BUFFER_SIZE];                
unsigned char rx_encode_frame[ENCODE_FRAME_SIZE];      // 44 =  First four bytes non-audio + 40 bytes audio
unsigned char tx_encode_frame[ENCODE_FRAME_SIZE];      
int tx_encode_frame_index = 0;
uint8_t rx_raw_audio_value = 127;                      
int adc_buffer_index = 0;
int16_t Sine1KHz[8] = { -21210 , -30000, -21210, 0 , 21210 , 30000 , 21210, 0 };  //int16_t 1KHz sine test tone
int Sine1KHz_index = 0;

// REPLACE WITH YOUR RECEIVER MAC Address (ESPNow)
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

FastAudioFIFO audio_fifo;

enum RadioState
{
  radio_standby, radio_rx, radio_tx 
};
volatile RadioState radio_state = RadioState::radio_tx;

struct CODEC2* codec2_state;

ButterworthFilter hp_filter(240, 8000, ButterworthFilter::ButterworthFilter::Highpass, 1);  //Implement a high pass 240Hz Butterworth Filter.
hw_timer_t* adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t codec2HandlerTask;


////////////////////////////////////End of Variables/////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////Start of timer function///////////////////////////////////////////////////////////////////////////////////


void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);                //Enter crital code without interruptions

       if (radio_state == RadioState::radio_tx)          // Microphone is open and active, Transmit Audio from microphone.
       {
        //Read the ADC and convert it's value from (0 - 4095) to (-32768 - 32767) = 16 x 4095 = 65520 total
        //adc_buffer[adc_buffer_index++] = (16 * adc1_get_raw(ADC1_CHANNEL_0)) - 32768;  // Input voice data from microphone and store it to int16_t array called adc_buffer
    
        //If you want to test with a 1KHz tone, comment the line above and uncomment the line below
    
        adc_buffer[adc_buffer_index++] = Sine1KHz[Sine1KHz_index++]; if (Sine1KHz_index >= 8) Sine1KHz_index = 0; //For testing without microphone.
       
        if (adc_buffer_index == ADC_BUFFER_SIZE) {      // When buffer is full i.e. adc_buffer array has 320 bytes
        adc_buffer_index = 0;

    //Transfer the buffer from adc_buffer to speech buffer
      memcpy((void*)speech, (void*)adc_buffer, 2 * ADC_BUFFER_SIZE);  // Store 320 bytes of microphone voice data array to int16_t array called speech
      
      // Notify run_codec2 task that the buffer is ready.
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken)
      {
        portYIELD_FROM_ISR();
      }
    }
  } else if (radio_state == RadioState::radio_rx) {       // Create audio & write to DAC
    
    int16_t v;                                            // Create variable v to store voice data
    
    if (audio_fifo.get(&v))                               // Get audio data and send it to DAC to play
      rx_raw_audio_value = (uint8_t)((v + 32768) / 256);  // Convert audio data to uint8_t (in 0-255 values) integer type variable from uint16_t type variable.
      dacWrite(lineOut, rx_raw_audio_value);              // Play stream from uint8_t integer type variable values (audio data)
    }
  portEXIT_CRITICAL_ISR(&timerMux); // exit critical code
}

////////////////////////////////////////End of timer function/////////////////////////////////////////////////////////////////////



///////////////////////////////////////////Start of setup////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200); 
  delay(500);
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
  
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC_PIN, ADC_ATTEN_DB_6);  //ADC 1 channel 0 (GPIO36).

  //Start the task that run the coder and decoder
  xTaskCreate(&run_codec2, "codec2_task", 30000, NULL, 5, &codec2HandlerTask);
  Serial.println("Codec2 encoder & decoder Started....");
  
  //Start a timer at 8kHz to sample the ADC and play the audio on the DAC.
  adcTimer = timerBegin(3, 500, true);            // 80 MHz / 500 = 160KHz MHz hardware clock
  timerAttachInterrupt(adcTimer, &onTimer, true); // Attaches the handler function to the timer 
  timerAlarmWrite(adcTimer, 20, true);            // Interrupts when counter == 20, 8.000 times a second
  timerAlarmEnable(adcTimer);                     // Activate it

  //Configure PTT input button
  pinMode(PTT_PIN, INPUT_PULLUP);

  //Set state 
  radio_state = RadioState::radio_rx;
  Serial.println("Waiting to send or receive audio packets...");
}

//////////////////////////////////////End of setup//////////////////////////////////////////////////////////////////

void loop() {
  
  if (digitalRead(PTT_PIN) == LOW || touchRead(12) < 50)
  {
    radio_state = RadioState::radio_tx;
    Serial.println("Transmitting audio packets...");        
  } else {
     radio_state = RadioState::radio_rx;
  }

delay(1);              //At least 1ms please!
}

//////////////////////////////////////End of loop//////////////////////////////////////////////////////////////////

void run_codec2(void* parameter) {  // Main Codec2 function

  codec2_state = codec2_create(CODEC2_MODE_1600);
  codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

  // Header, you have 4 bytes of header in each frame, you can use it for you protocol implementation
  // for instace indicate address of the transmiter and address of the desire receiver, etc.
  
  tx_encode_frame[0] = 0x00; tx_encode_frame[1] = 0x00; tx_encode_frame[2] = 0x00; tx_encode_frame[3] = 0x00;
  
  RadioState last_state = RadioState::radio_standby;
  while (1)
  {
    
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000)); //Wait until be notify or 1 second

    if (tcount != 0)                                                  //if the task was notified! 
    {
      //Init the tx_encode_frame_index if a trasmition start
      if (radio_state != last_state)
      {
        if (radio_state == RadioState::radio_tx)            // Trasnmitting - Microphone open and active
        { 
          tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE; // i.e. 4 bytes
          last_state = radio_state;} 
          for (int i = 0; i < ADC_BUFFER_SIZE; i++)
          speech[i] = (int16_t)hp_filter.Update((float)speech[i]);                            // Apply High Pass Filter
          codec2_encode(codec2_state, tx_encode_frame + tx_encode_frame_index, speech); // encode the 320 bytes(40ms) of speech frame into 8 bytes per ms
          tx_encode_frame_index += ENCODE_CODEC2_FRAME_SIZE;                            // increment (by 8 bytes) the pointer where the encoded frame must be saved 
          if (tx_encode_frame_index == ENCODE_FRAME_SIZE)                               // If it is the 5th time then we have a ready trasnmission frame of 44 bytes
          {
           tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE;                            // increment the pointer by 4 bytes
           esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) tx_encode_frame, sizeof(tx_encode_frame)); // Send message via ESP-NOW
          }
         }

         if (radio_state == RadioState::radio_rx) // receiving audio packet & storing it to buffer
         {
           for (int i = ENCODE_FRAME_HEADER_SIZE; i < ENCODE_FRAME_SIZE; i += ENCODE_CODEC2_FRAME_SIZE)  //Make a cycle to get each codec2 frame from the received frame
            {
              codec2_decode(codec2_state, output_buffer, rx_encode_frame + i);                           // Decode the codec2 frame and store decoded audio into int16_t array of 320 bytes called output buffer
              for (int g = 0; g < ADC_BUFFER_SIZE; g++) audio_fifo.put(output_buffer[g]);                // Add to the audio buffer the 320 samples resulting of the decode of the codec2 frame.
            }                
         }
      }
   }
}


/////////////////////////////////////Start of receive event handler function of ESPNow //////////////////////////////////////////////////////////////

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
  //Serial.print("bytes received: ");Serial.println(len);
  memcpy((void*)rx_encode_frame, (void*)data, len);  // Store 44 bytes  
  for (int i = 0; i < len; i++) {/* Serial.println(rx_encode_frame[i]);*/}
  radio_state = RadioState::radio_rx; //Serial.println("Receiving audio packets..."); //Set the state to radio_rx because we are receiving
  
 // Notify run_codec2 task that we have received a new packet.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    } 
}
