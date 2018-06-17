#include <FastLED.h>
#include <arduinoFFT.h>
#undef min
#undef max
#include <array>

arduinoFFT fft = arduinoFFT();

// LED strip
// konstanten
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 60;

// mic
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;

// led array
std::array<CRGB, NUM_LEDS> leds;

// DMA
#define ADCPIN A0
#define HWORDS 128
uint16_t adcbuf[HWORDS];

typedef struct {
  uint16_t btctrl;
  uint16_t btcnt;
  uint32_t srcaddr;
  uint32_t dstaddr;
  uint32_t descaddr;
} dmacdescriptor ;
volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));

static uint32_t chnl = 0;  // DMA channel
volatile uint32_t dmadone;

void DMAC_Handler() {
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  uint8_t active_channel;
  // disable irqs ?
  __disable_irq();
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  dmadone = DMAC->CHINTFLAG.reg;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
  __enable_irq();
  interrupt_counter++;
}


void dma_init() {
  // probably on by default
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC ;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC ;
  NVIC_EnableIRQ( DMAC_IRQn ) ;

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}

void adc_dma(void *rxdata,  size_t hwords) {
  uint32_t temp_CHCTRLB_reg;

  DMAC->CHID.reg = DMAC_CHID_ID(chnl);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << chnl));
  temp_CHCTRLB_reg = DMAC_CHCTRLB_LVL(0) |
                     DMAC_CHCTRLB_TRIGSRC(ADC_DMAC_ID_RESRDY) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHCTRLB.reg = temp_CHCTRLB_reg;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK ; // enable all 3 interrupts
  dmadone = 0;
  descriptor.descaddr = 0;
  descriptor.srcaddr = (uint32_t) &ADC->RESULT.reg;
  descriptor.btcnt =  hwords;
  descriptor.dstaddr = (uint32_t)rxdata + hwords * 2; // end address
  descriptor.btctrl =  DMAC_BTCTRL_BEATSIZE_HWORD | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID;
  memcpy(&descriptor_section[chnl], &descriptor, sizeof(dmacdescriptor));

  // start channel
  DMAC->CHID.reg = DMAC_CHID_ID(chnl);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

static __inline__ void ADCsync() __attribute__((always_inline, unused));
static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void adc_init() {
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
  ADC->AVGCTRL.reg = 0;       // Averaging mit 8 Samples zu 1
  ADC->SAMPCTRL.reg = 0x00;  ; //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT; // 48Mhz / 64 = 750kHz, 1samples = 6clocks => 750kHz / 6 = 125Khz. Mit Averaging -> 125kHz / 8 = 15.625kHz
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}

void setup() {
  // setup code
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  // keine ahnung obs auswirkung hat, meine PowerBank hat 2.1A darum hab ich mal 2A max eingestellt. Sollte die Helligkeit automatisch regeln wenn zuviele Leds zu hell eingestellt sind
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  // alle leds gleiche farbe
  CHSV color(0, 255, 255) ; // rot
  fill_solid(leds.data(), leds.size(), color);

  Serial.begin(9600);
  //analogWriteResolution(10);
  //analogWrite(A0, 64);  // test with DAC
  adc_init();
  dma_init();
}

std::array<double, HWORDS> samplesArray = {0};
std::array<double, HWORDS> complexArray = {0};

void loop() {
  
  adc_dma(adcbuf, HWORDS);
  uint32_t t_dma = micros();
  while (!dmadone) // await DMA done isr
  {

  }
  Serial.print("d ");
  Serial.println(interrupt_counter);
  for (int i = 0; i < HWORDS; i++)
  {
    samplesArray[i] = (3.3 / 1024.0 * adcbuf[i]) - (3.3/2); // scale to 12bit resolution (2^12 = 4096)
    complexArray[i] = 0;
  }
  
  /*for (int i = 0; i < HWORDS; i++)
  {
    Serial.print("0 4096 ");
    Serial.println(adcbuf[i]);
  }*/
   
  fft.Windowing(samplesArray.data(),samplesArray.size(), FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  fft.Compute(samplesArray.data(), complexArray.data(), samplesArray.size(), FFT_FORWARD);
  fft.ComplexToMagnitude(samplesArray.data(), complexArray.data(), samplesArray.size());
  // in samples stehen jetzt die fft werte (index 0 bis samples->size()/2)
  //double peak = fft.MajorPeak(samplesArray.data(), samplesArray.size(), 15625);
  //Serial.print(peak); Serial.println(" Hz");

    for(int i = 0; i < 60; i++)
    { 
        int binIndex = i+1;
        double val = samplesArray[i] / 10 * 255;
        if(val > 255) {
          val = 255;
        }
        if(val < 1)
        {
          val += 1;
        }
        Serial.print(val);
        Serial.print(" ");
        CHSV color(h, 255, static_cast<uint8_t>(val)) ; 
        leds[i] = color;
    }
    FastLED.show();
}
