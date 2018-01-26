#include <Adafruit_ZeroDMA.h>
#include <FastLED.h>
#include <arduinoFFT.h>
#undef min
#undef max
#include <array>

// LED
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 60;
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

// fft
arduinoFFT fft;

void dma_callback(Adafruit_ZeroDMA *dma) {
  transfer_is_done = true;
}

//This is the interrupt service routine (ISR) that is called 
//if an ADC measurement falls out of the range of the window 
void ADC_Handler() {
  Serial.println("sample out of scope");
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
  Serial.println("Configuring ADC");
  adc_setup();
  
  Serial.println("Configuring DMA trigger");
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
  Serial.println("Adding callback");
  // register_callback() can optionally take a second argument
  // (callback type), default is DMA_CALLBACK_TRANSFER_DONE
  myDMA.setCallback(dma_callback);
}

void setup() {
  Serial.begin(9600);
  while(!Serial); // Wait for Serial monitor before continuing
  
  dma_setup();
  
  fft = arduinoFFT();

  // leds
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  // keine ahnung obs auswirkung hat, meine PowerBank hat 2.1A darum hab ich mal 2A max eingestellt. Sollte die Helligkeit automatisch regeln wenn zuviele Leds zu hell eingestellt sind
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  // alle leds gleiche farbe
  CHSV color(0, 255, 255) ; // rot
  fill_solid(leds.data(), leds.size(), color);
}

uint8_t h = 0;
void audio_spectrum()
{
  /*for(int i =  0; i < SAMPLES; i++){
  Serial.print((adc_buffer[i] * 3.3 / 1024)  - (3.3/2));
  Serial.print("\t");
  }*/
  
  std::array<double, SAMPLES > samplesArray = {0};
  std::array<double, SAMPLES > complexArray = {0};
  for (int i = 0; i < SAMPLES; i++)
  {
    samplesArray[i] = (3.3 / 1024.0 * adc_buffer[i]) - (3.3/2); // scale to 10bit resolution (2^10 = 1024) and set zero point to "0" (instead of 1.65V)
    complexArray[i] = 0;
  }
  
  fft.Windowing(samplesArray.data(),samplesArray.size(), FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  fft.Compute(samplesArray.data(), complexArray.data(), samplesArray.size(), FFT_FORWARD);
  fft.ComplexToMagnitude(samplesArray.data(), complexArray.data(), samplesArray.size());
  // in samples stehen jetzt die fft werte (index 0 bis samples->size()/2)
  
    // print out
  for(int i = 0; i < 60; i++)
  { 
      double val = samplesArray[i] / 10 * 255;
      if(val > 255) {
        val = 255;
      }
      CHSV color(h, 255, static_cast<uint8_t>(val)) ; 
      leds[i] = color;
      //leds[59-i] = color;
  }
  FastLED.show();
}

void loop() {
  myDMA_status = myDMA.startJob();
  while(!transfer_is_done); // Chill until DMA transfer completes
  myDMA.printStatus(myDMA_status); // Results of start_transfer_job()
  if(myDMA_status == DMA_STATUS_BUSY)
  {
    myDMA.abort();
    for(int i = 0; i < SAMPLES; i++)
    {
      adc_buffer[i] = 0;
    }
  }
  else
  {
    audio_spectrum();
    h += 1; 
  }
}
