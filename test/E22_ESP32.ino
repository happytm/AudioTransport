#include <Arduino.h>
#include <driver/adc.h>
#include <codec2.h>                  // Thanks to https://github.com/deulis/ESP32_Codec2
#include <ButterworthFilter.h>       // In the codec2 folder in the library folder
#include <FastAudioFIFO.h>           // In the codec2 folder in the library folder
#include "LoRa_E22.h"

#define ADC_PIN ADC1_CHANNEL_0          // ADC 1 channel 0 (GPIO36)
#define lineOut 25
#define PTT_PIN 12

#define ADC_BUFFER_SIZE 320             // 40ms of voice in 8KHz sampling frequency
#define ENCODE_FRAME_SIZE 44            // 44 =  First four bytes non-audio + 40 bytes audio
#define ENCODE_CODEC2_FRAME_SIZE 8      // 8 bytes per packets x 5 packets = 40 bytes of audio packet
#define ENCODE_FRAME_HEADER_SIZE 4      // 4 bytes of non-audio data

int16_t adc_buffer[ADC_BUFFER_SIZE];                   // 320 = 40ms of voice in 8KHz sampling frequency
int16_t speech[ADC_BUFFER_SIZE];                       // 320 = 40ms of voice in 8KHz sampling frequency
int16_t output_buffer[ADC_BUFFER_SIZE];                // 320 = 40ms of voice in 8KHz sampling frequency
uint8_t rx_encode_frame[ENCODE_FRAME_SIZE];      // 44 =  First four bytes non-audio + 40 bytes audio
uint8_t tx_encode_frame[ENCODE_FRAME_SIZE];      // 44 bytes

int tx_encode_frame_index = 0;
uint8_t rx_raw_audio_value = 127;                      
int adc_buffer_index = 0;

// Wemos Lolin32 board
LoRa_E22 Lora(&Serial2, 4, 18, 2); //  TXD=16 RXD=17 AUX=4 M0=18 M1=2

FastAudioFIFO audio_fifo;

enum RadioState
{
  radio_standby, radio_rx, radio_tx 
};
volatile RadioState radio_state = RadioState::radio_tx;

//The codec2 
struct CODEC2* codec2_state;

//Implement a high pass 240Hz Butterworth Filter.
ButterworthFilter hp_filter(240, 8000, ButterworthFilter::ButterworthFilter::Highpass, 1);
hw_timer_t* adcTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t codec2HandlerTask;

struct Message {
    uint8_t audio[44];
} message;
////////////////////////////////////End of Variables///////////////////////////////////////////////////////////////////////////

// onReceive event handler
void onReceive() 
{
    radio_state = RadioState::radio_rx;
    //Serial.println("Waiting for audio packets...");

    if (Lora.available()>1) {
    // read the String message
    #ifdef ENABLE_RSSI
    ResponseStructContainer rsc = Lora.receiveMessageRSSI(sizeof(Message));
    #else
    ResponseStructContainer rsc = Lora.receiveMessage(sizeof(Message));
    #endif  

    // Is something goes wrong print error
  if (rsc.status.code!=1){
    //Serial.println(rsc.status.getResponseDescription());
  
  } else {
    
    // Print the data received
    //Serial.println(rsc.status.getResponseDescription());
    
    struct Message message = *(Message*) rsc.data;
    for (int i = 0; i < ENCODE_FRAME_SIZE; i++) 
    {
      //Serial.println(message.audio[i]);
      rx_encode_frame[i] = message.audio[i];
    }
  }
    // Notify run_codec2 task that we have received a new packet.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(codec2HandlerTask, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR();
    }
  }
}

////////////////////////////////////////Start of timer function/////////////////////////////////////////////////////////////////////

//int16_t 1KHz sine test tone
int16_t Sine1KHz[8] = { -21210 , -30000, -21210, 0 , 21210 , 30000 , 21210, 0 };
int Sine1KHz_index = 0;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);                //Enter crital code without interruptions

/////////////////////////////////Transmit Audio from microphone////////////////////////////////////////////////////////////////////////////

  if (radio_state == RadioState::radio_tx)          // Microphone is open and active
  {
    //Read the ADC and convert it's value from (0 - 4095) to (-32768 - 32767) = 16 x 4095 = 65520 total
    //adc_buffer[adc_buffer_index++] = (16 * adc1_get_raw(ADC1_CHANNEL_0)) - 32768;  // Input voice data from microphone and store it to int16_t array called adc_buffer
    
    //If you want to test with a 1KHz tone, comment the line above and descomment the three lines below
    
    adc_buffer[adc_buffer_index++] = Sine1KHz[Sine1KHz_index++];
    if (Sine1KHz_index >= 8)
      Sine1KHz_index = 0;
       
    //When buffer is full
    if (adc_buffer_index == ADC_BUFFER_SIZE) {      // i.e. adc_buffer array has 320 bytes
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
  }
  
////////////////////////Create audio & write to DAC///////////////////////////////////////////////////////////////////////////
  
  else if (radio_state == RadioState::radio_rx)
  {
    
    int16_t v;
    
    if (audio_fifo.get(&v))                               // Get audio data and send it to DAC to play
      rx_raw_audio_value = (uint8_t)((v + 32768) / 256);  // Store audio data in 0-255 values to uint8_t integer type variable
      dacWrite(lineOut, rx_raw_audio_value); // Play stream from uint8_t integer type variable values (audio data)
      //Serial.print(rx_raw_audio_value);
  }
  portEXIT_CRITICAL_ISR(&timerMux); // exit critical code
}

////////////////////////////////////////End of timer function/////////////////////////////////////////////////////////////////////

void run_codec2(void* parameter)          // This function is called from setup
{
  //Init codec2
  codec2_state = codec2_create(CODEC2_MODE_1600);
  codec2_set_lpc_post_filter(codec2_state, 1, 0, 0.8, 0.2);

  // Header, you have 4 bytes of header in each frame, you can use it for you protocol implementation
  // for instace indicate address of the transmiter and address of the desire receiver, etc.
  
  tx_encode_frame[0] = 0x00;
  tx_encode_frame[1] = 0x00;
  tx_encode_frame[2] = 0x00;
  tx_encode_frame[3] = 0x00;
  
  RadioState last_state = RadioState::radio_standby;
  while (1)
  {
    //Wait until be notify or 1 second
    uint32_t tcount = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));

    if (tcount != 0) //if the task was notified! 
    {
      //Init the tx_encode_frame_index if a trasmition start
      if (radio_state != last_state)
      {
        if (radio_state == RadioState::radio_tx)
        {
          tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE;  // i.e. 4 bytes
        }
        last_state = radio_state;
      }

/////////////////Trasmitting audio data from microphone/////////////////////////////////////////////////////////
      
      if (radio_state == RadioState::radio_tx)    // Trasnmitting - Microphone open and active
      {        
        //Apply High Pass Filter
        for (int i = 0; i < ADC_BUFFER_SIZE; i++)
          speech[i] = (int16_t)hp_filter.Update((float)speech[i]);
          
        //encode the 320 bytes(40ms) of speech frame into 8 bytes per ms
        codec2_encode(codec2_state, tx_encode_frame + tx_encode_frame_index, speech); 
        
        //increment the pointer where the encoded frame must be saved
        tx_encode_frame_index += ENCODE_CODEC2_FRAME_SIZE;   // 8 bytes

         //If it is the 5th time then we have a ready trasnmission frame
        if (tx_encode_frame_index == ENCODE_FRAME_SIZE)      // 44 bytes
        {
          
          tx_encode_frame_index = ENCODE_FRAME_HEADER_SIZE;  // increment the pointer at 4 bytes
         }
       }

//////////////////////////////////////receiving audio packet & storing it to buffer///////////////////////////////////////////////////////
     
      if (radio_state == RadioState::radio_rx) // Receiving
      {        
        //Make a cycle to get each codec2 frame from the received frame
        for (int i = ENCODE_FRAME_HEADER_SIZE; i < ENCODE_FRAME_SIZE; i += ENCODE_CODEC2_FRAME_SIZE)
        {
          //Decode the codec2 frame
          codec2_decode(codec2_state, output_buffer, rx_encode_frame + i);  // decoded audio stored to int16_t array of 320 bytes called output buffer
          // Add to the audio buffer the 320 samples resulting of the decode of the codec2 frame.
          for (int g = 0; g < ADC_BUFFER_SIZE; g++)
            audio_fifo.put(output_buffer[g]);                 
        }                 
      }
    }
  }
}

///////////////////////////////////////////Start of setup////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600); 
  delay(500);
  
  //Configure PTT input button
  pinMode(PTT_PIN, INPUT_PULLUP);
  
  Lora.begin();
  
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

  //Set state 
  radio_state = RadioState::radio_rx;
  Serial.println("Waiting to send or receive audio packets...");
}

//////////////////////////////////////End of setup//////////////////////////////////////////////////////////////////

void loop() {
  
  if (digitalRead(PTT_PIN) == LOW)
  {
    radio_state = RadioState::radio_tx;
    //Serial.println("Transmitting audio packets...");

    for (int i = 0; i < ENCODE_FRAME_SIZE; i++)
       message.audio[i] =  tx_encode_frame[i];
          
       // Send message
       ResponseStatus rs = Lora.sendMessage(&message, sizeof(Message));
       // Check If there is some problem of succesfully send
       //Serial.println(rs.getResponseDescription());
  
  } else {
     onReceive();  
  }
  delay(1);              //At least 1ms please!
}                        // End of Loop
