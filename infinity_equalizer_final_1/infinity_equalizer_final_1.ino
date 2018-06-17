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
// has to be even
constexpr static int NUM_LEDS = 106; // 108
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

// double global_max_input_value = 0;  // measuring: the highest input of all samples ever collected
// int timer_mv = micros();            // time measuring 
void audio_spectrum()
{ 
  std::array<double, SAMPLES> samplesArray = {0};
  std::array<double, SAMPLES> complexArray = {0};

  for (int i = 0; i < SAMPLES; i++)
  {
    samplesArray[i] = (VCC_microphone / 1024.0 * adc_buffer[i]) - (VCC_microphone/2); // scale to 10bit resolution (2^10 = 1024) and set zero point to "0" (instead of 1.65V)
    complexArray[i] = 0;
  }
  
  fft.Windowing(samplesArray.data(),samplesArray.size(), FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  fft.Compute(samplesArray.data(), complexArray.data(), samplesArray.size(), FFT_FORWARD);
  fft.ComplexToMagnitude(samplesArray.data(), complexArray.data(), samplesArray.size());
  // samplesArray know contains the calculated fft values

  // used for color and brightness scaling, you can play with them
  double input_min = 0;
  double input_max = 3; // 13
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

    // scale hue 
    if(hue > 128 && hue < 177)
    {
      // blue colors require less brightness
      scaled_0_to_1 = - pow((x_0_to_1 -1), 2) + 1;
    }
    else
    {
      scaled_0_to_1 = - pow((x_0_to_1 -1), 4) + 1;
    }
    
    // scale back to values from 0 to 255
    double value = scaled_0_to_1 * (output_max - output_min);

    // value should not go over max output
    if(value > output_max) {
      value = output_max;
    }
    // turn off the "dark" LEDs
    if( value < 18)
    {
      value = 0;
    }
    
    // create color
    CHSV color(hue, 255, static_cast<uint8_t>(value));
    setLedColors(i, color);
  }
}

void rainbow_colors(int &value_counter)
{
  for(int i = 0; i < (NUM_LEDS/2); i++)
  {
      double col = 255/(NUM_LEDS/2) * i;
      CHSV color(static_cast<uint8_t>(col), 255, value_counter);
      setLedColors(i, color);
  }
}

void setLedColors(int &index, CHSV &color)
{
  leds[index] = color;                // set color to corresponding LED
  leds[(NUM_LEDS-1)-index] = color;   // this "mirrors" the LED to the other side
}

int rainbow_timer = micros();
int loop_iteration_counter = 0;
int rainbow_value_counter = 0;
bool color_flag = true;
bool isQuiet = false;
bool time_up = false;
void loop() {
  myDMA_status = myDMA.startJob();  // start dma job
  while(!transfer_is_done);         // chill until DMA transfer completes
  if(myDMA_status == DMA_STATUS_OK)
  {
    isQuiet = *std::max_element(std::begin(adc_buffer), std::end(adc_buffer)) < 520; // silent -> 515, with ambient music almost every sample over 520
    time_up = (micros() - rainbow_timer) > 10000000; // timer over 10 seconds

    // reset timer whenever music is playing
    if(!isQuiet)
    {
      rainbow_timer = micros();
    }

    if(isQuiet && time_up)
    {
      // calculate color with rainbow style
      rainbow_colors(rainbow_value_counter);
    }
    else
    {
      // calculate colors/brightness based on FFT
      audio_spectrum();
    }
    FastLED.show();
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
  
  if(loop_iteration_counter % 30 == 0)
  {
    if(color_flag)
    {
      if(rainbow_value_counter < 255)
      {
        rainbow_value_counter++;
      }
      else
      {
        color_flag = false;
        rainbow_value_counter--;
      } 
    }
    else
    {
      if(rainbow_value_counter > 0)
      {
        rainbow_value_counter--;
      }
      else
      {
        color_flag = true;
        rainbow_value_counter++;
      } 
    }
  }
  loop_iteration_counter++;
}
