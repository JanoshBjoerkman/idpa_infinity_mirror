#include <Adafruit_ZeroDMA.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#undef min
#undef max
#include <array>
#include <iterator>
#include <algorithm>
#include <math.h>  

// for reference (microphone) see: https://learn.adafruit.com/adafruit-microphone-amplifier-breakout
constexpr static double VCC_microphone = 3.3;

// LEDs
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 108;
// led array
std::array<CRGB, NUM_LEDS> leds;

// DMA
Adafruit_ZeroDMA myDMA;                 // see: https://github.com/adafruit/Adafruit_ZeroDMA
ZeroDMAstatus    myDMA_status;          // DMA status codes returned by some functions
volatile bool transfer_is_done = true;  // done yet?
#define ADCPIN A0     // microphone out
#define SAMPLES 128   // number of samples per run
uint16_t adc_buffer[SAMPLES], 
          buffer_being_filled = 0,  // Index of 'filling' buffer
          buffer_value        = 0;  // Value of fill
DmacDescriptor *desc;               // DMA descriptor address (so we can change contents)
// dma callback function 
void dma_callback(Adafruit_ZeroDMA *dma) {
  transfer_is_done = true;
}

// FFT
arduinoFFT fft;   // see: https://github.com/kosme/arduinoFFT

// this is the interrupt service routine (ISR) that is called 
// if an ADC measurement falls out of the range of the window 
void ADC_Handler() {
  // Serial.println("sample out of scope");
  ADC->INTFLAG.reg = ADC_INTFLAG_WINMON;  // need to reset interrupt
}

// predeclaration of ADC-sync function
static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void adc_setup()
{
  analogRead(ADCPIN);  // do some pin init  pinPeripheral()
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  ADCsync();
  //ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;    // gain select as 1X
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;    // default
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // zero-level at 3.3V / 2 -> 1.65 V
  ADCsync();    // see chaoter 31.6.16 for reference: https://cdn.sparkfun.com/datasheets/Dev/Arduino/Boards/Atmel-42181-SAM-D21_Datasheet.pdf
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber;
  ADCsync();
  ADC->AVGCTRL.reg = 0;       // no averaging
  ADC->SAMPCTRL.reg = 0x00;   //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  // set prescaler and 10bit resolution: 48MHz / (512 * 6 clock cycles) -> 15625Hz samplerate
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT;
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;   // finaly enable 
  ADCsync();
}

void dma_setup(){
  // see: https://github.com/adafruit/Adafruit_ZeroDMA
  //Serial.println("Configuring DMA trigger");
  myDMA.setTrigger(ADC_DMAC_ID_RESRDY);     // adc result ready triggers dma
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
  myDMA.setCallback(dma_callback);  // function which is executet when dma finished
}

void setup() {
  Serial.begin(9600);   // set baudrate
  //while(!Serial);     // wait for Serial monitor before continuing
  
  //Serial.println("Configuring ADC");
  adc_setup();
  dma_setup();
  
  fft = arduinoFFT();

  // configure LEDs
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2400);  // regulates brightness

  CHSV color(0, 255, 255) ;                     // red
  fill_solid(leds.data(), leds.size(), color);  // set all LEDs to same color
}

double global_max_input_value = 0;  // mesuring: the high
int timer_mv = micros();            // measuring 
void audio_spectrum()
{  
  std::array<double, SAMPLES > samplesArray = {0};
  std::array<double, SAMPLES > complexArray = {0};
  for (int i = 0; i < SAMPLES; i++)
  {
    samplesArray[i] = (VCC_microphone / 1024.0 * adc_buffer[i]) - (VCC_microphone/2); // scale to 10bit resolution (2^10 = 1024) and set zero point to "0" (instead of 1.65V)
    complexArray[i] = 0;
  }
  
  fft.Windowing(samplesArray.data(),samplesArray.size(), FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  fft.Compute(samplesArray.data(), complexArray.data(), samplesArray.size(), FFT_FORWARD);
  fft.ComplexToMagnitude(samplesArray.data(), complexArray.data(), samplesArray.size());
  // samplesArray know contains the calculated fft values

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

  // used for color and brightness scaling, you can play with them
  double input_min = 0;
  double input_max = 13;
  double output_max = 255;
  double output_min = 0;

  // print out
  // WARNING: NUM_LEDS has to be even
  for(int i = 0; i < NUM_LEDS / 2; i++)
  {
    // get the color first, will be used later 
    uint8_t hue = ((255/(NUM_LEDS / 2))*((NUM_LEDS / 2)-i) - 50); // color scaled to FastLED rainbow hue chart. begins with pink/blue
    // see color chart for reference: https://github.com/FastLED/FastLED/wiki/FastLED-HSV-Colors
    
    double x_0_to_1 = 0; 
    double scaled_0_to_1 = 0;
    if((samplesArray[i] - input_min) > 0)
    {
      // scale the samples to values from 0 to 1
      x_0_to_1 = (samplesArray[i] - input_min) / (input_max - input_min);
    }
    /*
    // red - orange
    if(hue < 32) 
    {
      //double sqrt_05 = sqrt(x_0_to_1)+ 0.05;
      //scaled_0_to_1 = (sqrt_05 > 0) ? sqrt_05 : 0;
      scaled_0_to_1 = - pow((x_0_to_1 -1), 4) + 1; 
    }
    // orange - yellow
    if(hue >= 32 && hue <= 64)
    {
      //double sqrt_08 = sqrt(x_0_to_1)+ 0.08;
      //scaled_0_to_1 = (sqrt_08 > 0) ? sqrt_08 : 0; 
      scaled_0_to_1 = pow((x_0_to_1 -1), 5) + 1;
    }
    // yellow - aqua
    if(hue > 64 && hue <= 128)
    {
      //double sqrt_07 = sqrt(x_0_to_1)+ 0.07;
      //scaled_0_to_1 = (sqrt_07 > 0) ? sqrt_07 : 0; 
      scaled_0_to_1 = pow((x_0_to_1 -1), 5) + 1;
    }
    // aqua - purple
    if(hue > 128 && hue < 180)
    {
      //double sqrt_n03 = sqrt(x_0_to_1) - 0.14;
      //scaled_0_to_1 = (sqrt_n03 > 0) ? sqrt_n03 : 0;
      scaled_0_to_1 = - pow((x_0_to_1 -1), 2) + 1; 
    }*/
    /*if(x_0_to_1 - 0.1 > 0)
    {
     scaled_0_to_1 = 1.1 * sqrt(x_0_to_1 - 0.1); // see: https://www.wolframalpha.com/input/?i=1.1*sqrt(x-0.1)+for+x+%3D+0+to+1
    }
    else
    {
      scaled_0_to_1 = 0;
    }*/
    //scaled_0_to_1 = sqrt(x_0_to_1);
    //scaled_0_to_1 = pow((x_0_to_1 -1), 3) + 1;
    //scaled_0_to_1 = - pow((x_0_to_1 -1), 2) + 1;
    //scaled_0_to_1 = x_0_to_1;
    
    if(hue > 128 && hue < 177)
    {
      // blue colors require less brightness
      scaled_0_to_1 = - pow((x_0_to_1 -1), 2) + 1;
    }
    else
    {
      scaled_0_to_1 = pow((x_0_to_1 -1), 3) + 1;
    }
    // scale back to values from 0 to 255
    double value = scaled_0_to_1 * (output_max - output_min);
    
    // value should not go over 255
    if(value > 255) {
      value = 255;
    }
    // turn off the "dark" LEDs
    if( value < 15)
    {
      value = 0;
    }
    // create color
    CHSV color(hue, 255, static_cast<uint8_t>(value));
    leds[i] = color;                // set color to corresponding LED
    leds[(NUM_LEDS-1)-i] = color;   // this "mirrors" the LED to the other side
  }
  // display LEDs
  FastLED.show();
}

void loop() {
  // uint8_t time_all = micros();
  // uint8_t time_dma = micros();
  myDMA_status = myDMA.startJob();  // start dma job
  while(!transfer_is_done);         // chill until DMA transfer completes
  // time_dma = micros() - time_dma;
  // Serial.print(time_dma);
  // Serial.println(" us -> dma");
  // myDMA.printStatus(myDMA_status); // Results of start_transfer_job()
  if(myDMA_status == DMA_STATUS_OK)
  {
    // uint8_t time_fft = micros();
    audio_spectrum();   // fft, calculate colors/brightness, show LEDs
    // time_fft = micros() - time_fft;
    // Serial.print(time_fft);
    // Serial.println(" us -> fft");
    // time_all = micros() - time_all; 
    // Serial.print(time_all);
    // Serial.println(" us -> time all");
  }
  else
  {
    // something went wrong
    myDMA.abort();    // cancel dma job
    for(int i = 0; i < SAMPLES; i++)
    {
      adc_buffer[i] = 0;  // clear ADC values
    }
  }
}
