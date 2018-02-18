#include <FHT.h>
#include <Adafruit_ZeroDMA.h>
#include <FastLED.h>
#undef min
#undef max
#include <array>
#include <iterator>
#include <algorithm>
#include <math.h>  

// FHT, see: http://wiki.openmusiclabs.com/wiki/FHTFunctions
#define LOG_OUT 1
#define LIN_OUT8 1 // turn on output array of FHT
#define FHT_N 128 // set to 128 / 2 bins -> FHT
#define SCALE 255 // scaling factor 8b resolution
unsigned char fht_lin_out8[FHT_N/2];

// microphone see: https://learn.adafruit.com/adafruit-microphone-amplifier-breakout
constexpr static double VCC_microphone = 3.3;

// LEDC
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 108;
// led array
std::array<CRGB, NUM_LEDS> leds;

// DMA
Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    myDMA_status; // DMA status codes returned by some functions
volatile bool transfer_is_done = true; // Done yet?
#define ADCPIN A0
#define SAMPLES 128
uint16_t adc_buffer[SAMPLES],
          buffer_being_filled = 0, // Index of 'filling' buffer
          buffer_value        = 0; // Value of fill

DmacDescriptor *desc; // DMA descriptor address (so we can change contents)

void dma_callback(Adafruit_ZeroDMA *dma) {
  transfer_is_done = true;
}

//This is the interrupt service routine (ISR) that is called 
//if an ADC measurement falls out of the range of the window 
void ADC_Handler() {
  // Serial.println("sample out of scope");
  ADC->INTFLAG.reg = ADC_INTFLAG_WINMON; //Need to reset interrupt
}

static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void adc_setup()
{
  analogRead(ADCPIN);  // do some pin init  pinPeripheral()
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  ADCsync();
  //ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  //ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // default
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 0 Pegel bei 3.3V / 2 -> 1.65 V
  ADCsync();    //  ref 31.6.16
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber;
  ADCsync();
  ADC->AVGCTRL.reg = 0;       // no averaging
  ADC->SAMPCTRL.reg = 0x00;  ; //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT; // 48Mhz / 64 = 750kHz, 1samples = 6clocks => 750kHz / 6 = 125Khz. Mit Averaging -> 125kHz / 8 = 15.625kHz
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}

void dma_setup(){
  //Serial.println("Configuring DMA trigger");
  myDMA.setTrigger(ADC_DMAC_ID_RESRDY);
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  myDMA_status = myDMA.allocate();
  myDMA.printStatus(myDMA_status);
  desc = myDMA.addDescriptor(
    (void *)(&REG_ADC_RESULT),  // move data from here
    adc_buffer,                 // to here
    SAMPLES,                    // this many...
    DMA_BEAT_SIZE_HWORD,        // bytes/hword/words
    false,                      // increment source addr?
    true);                      // increment dest addr?
  //Serial.println("Adding callback");
  // register_callback() can optionally take a second argument
  // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
  myDMA.setCallback(dma_callback);
}

void setup() {
  Serial.begin(9600);
  //while(!Serial); // Wait for Serial monitor before continuing
  
  //Serial.println("Configuring ADC");
  adc_setup();
  dma_setup();

  // leds
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  // keine ahnung obs auswirkung hat, meine PowerBank hat 2.1A darum hab ich mal 2A max eingestellt. Sollte die Helligkeit automatisch regeln wenn zuviele Leds zu hell eingestellt sind
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2400);
}

double global_max_input_value = 0;
int timer_mv = micros(); 
void audio_spectrum()
{
  /*for(int i =  0; i < SAMPLES; i++){
  Serial.print((adc_buffer[i] * VCC_microphone / 1024)  - (VCC_microphone/2));
  Serial.print("\t");
  }*/
  
  for (int i = 0; i < SAMPLES; i++)
  {
    fht_input[i] = (VCC_microphone / 1024.0 * adc_buffer[i]) - (VCC_microphone/2); // scale to 10bit resolution (2^10 = 1024) and set zero point to "0" (instead of 1.65V)
  }
  cli();
  //fft
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_lin8(); // take the output of the fht
  sei();
  /*double max_input_value = *std::max_element(samplesArray.begin(), samplesArray.end());
  if(global_max_input_value < max_input_value)
  {
    global_max_input_value = max_input_value;
  }
  if(micros()-timer_mv > 1000000)
  {
    timer_mv = micros();
    global_max_input_value = max_input_value;
  }
  Serial.println(max_input_value);
  Serial.println(global_max_input_value);*/
  
  // print out
  // WARNING: NUM_LEDS has to be even
  for(int i = 0; i < NUM_LEDS / 2; i++)
  {/*
    double input_min = 0;
    double input_max = 18;
    double output_max = 255;
    double output_min = 0;

    double x_0_to_1 = 0;
    if((fht_lin_out8[i] - input_min) > 0)
    {
      x_0_to_1 = (samplesArray[i] - input_min) / (input_max - input_min);
    }
    double scaled_0_to_1 = 0;
    scaled_0_to_1 = sqrt(2 * x_0_to_1);*/
    uint8_t value = fht_lin_out8[i]; //scaled_0_to_1 * (output_max - output_min);
    uint8_t hue = ((255/(NUM_LEDS / 2))*((NUM_LEDS / 2)-i) - 50);         // color scaled to FastLED rainbow hue chart. begins with pink/blue
    
    Serial.println(fht_lin_out8[i]);
    // value should not go over 255
    if(value > 255) {
      value = 255;
    }
    
    // turn off the "dark" leds
    if( value < 40)
    {
      //value = 0;
    }
    CHSV color(hue, 255, value);//static_cast<uint8_t>(value)) ; 
    leds[i] = color;
    leds[(NUM_LEDS-1)-i] = color;
  }
  FastLED.show();
}

void loop() {
  //uint8_t time_all = micros();
  //uint8_t time_dma = micros();
  myDMA_status = myDMA.startJob();
  while(!transfer_is_done); // Chill until DMA transfer completes
  //time_dma = micros() - time_dma;
  //Serial.print(time_dma);
  //Serial.println(" us -> dma");
  //myDMA.printStatus(myDMA_status); // Results of start_transfer_job()
  if(myDMA_status == DMA_STATUS_OK)
  {
    //uint8_t time_fft = micros();
    audio_spectrum();
    //time_fft = micros() - time_fft;
    //Serial.print(time_fft);
    //Serial.println(" us -> fft");
    //time_all = micros() - time_all; 
    //Serial.print(time_all);
    //Serial.println(" us -> time all");
  }
  else
  {
    myDMA.abort();
    for(int i = 0; i < SAMPLES; i++)
    {
      adc_buffer[i] = 0;
    }
  }
}
