/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <FastLED.h>
#include <arduinoFFT.h>
#undef min
#undef max
#include <array>

//Beginning of Auto generated function prototypes by Atmel Studio
void DMAC_Handler();
void dma_init();
void adc_dma(void* rxdata, size_t hwords);
void ADCsync();
void adc_init();
//End of Auto generated function prototypes by Atmel Studio

arduinoFFT fft;

// LED strip
// konstanten
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 60;

// led array
std::array<CRGB, NUM_LEDS> leds;

// DMA
#define ADCPIN A0
#define HWORDS 256        // number of samples
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

// seen on: https://www.freertos.org/Debugging-Hard-Faults-On-Cortex-M-Microcontrollers.html
extern "C" {
void HardFault_HandlerC(unsigned long *hardfault_args){
  volatile unsigned long stacked_r0 ;
  volatile unsigned long stacked_r1 ;
  volatile unsigned long stacked_r2 ;
  volatile unsigned long stacked_r3 ;
  volatile unsigned long stacked_r12 ;
  volatile unsigned long stacked_lr ;
  volatile unsigned long stacked_pc ;
  volatile unsigned long stacked_psr ;
  volatile unsigned long _CFSR ;
  volatile unsigned long _HFSR ;
  volatile unsigned long _DFSR ;
  volatile unsigned long _AFSR ;
  volatile unsigned long _BFAR ;
  volatile unsigned long _MMAR ;
 
  stacked_r0 = ((unsigned long)hardfault_args[0]) ;
  stacked_r1 = ((unsigned long)hardfault_args[1]) ;
  stacked_r2 = ((unsigned long)hardfault_args[2]) ;
  stacked_r3 = ((unsigned long)hardfault_args[3]) ;
  stacked_r12 = ((unsigned long)hardfault_args[4]) ;
  stacked_lr = ((unsigned long)hardfault_args[5]) ;
  stacked_pc = ((unsigned long)hardfault_args[6]) ;
  stacked_psr = ((unsigned long)hardfault_args[7]) ;
 
  // Configurable Fault Status Register
  // Consists of MMSR, BFSR and UFSR
  _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;
 
  // Hard Fault Status Register
  _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;
 
  // Debug Fault Status Register
  _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;
 
  // Auxiliary Fault Status Register
  _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;
 
  // Read the Fault Address Registers. These may not contain valid values.
  // Check BFARVALID/MMARVALID to see if they are valid values
  // MemManage Fault Address Register
  _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
  // Bus Fault Address Register
  _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;
 
  __asm("BKPT #0\n") ; // Break into the debugger
}
}
void HardFault_Handler()
{
 __asm volatile (
    " movs r0,#4       \n"
    " movs r1, lr      \n"
    " tst r0, r1       \n"
    " beq _MSP         \n"
    " mrs r0, psp      \n"
    " b _HALT          \n"
  "_MSP:               \n"
    " mrs r0, msp      \n"
  "_HALT:              \n"
    " ldr r1,[r0,#20]  \n"
    " b HardFault_HandlerC \n"
    " bkpt #0          \n"
  );
}

void DMAC_Handler() {
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  uint8_t active_channel;
  // disable irqs ?
  __disable_irq();
  active_channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);
  dmadone = DMAC->CHINTFLAG.reg;
  if(dmadone == 1)
  {
	// error 
	Serial.println("error");
  }
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_SUSP;
  __enable_irq();
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

__inline__ void ADCsync() __attribute__((always_inline, unused));
void   ADCsync() {
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
  ADC->AVGCTRL.reg = 0;       // no averaging
  ADC->SAMPCTRL.reg = 0x00;  ; //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT; // 48Mhz / 64 = 750kHz, 1samples = 6clocks => 750kHz / 6 = 125Khz. Mit Averaging -> 125kHz / 8 = 15.625kHz
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
}

// uint32_t ff_buffer[7000];

void setup() {
  // setup code
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  // keine ahnung obs auswirkung hat, meine PowerBank hat 2.1A darum hab ich mal 2A max eingestellt. Sollte die Helligkeit automatisch regeln wenn zuviele Leds zu hell eingestellt sind
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  // alle leds gleiche farbe
  CHSV color(0, 255, 0) ; // rot
  fill_solid(leds.data(), leds.size(), color);

  Serial.begin(9600);
  analogWriteResolution(10);
  // analogWrite(A0, 64);  // test with DAC
  adc_init();
  dma_init();
  fft = arduinoFFT();
  /*
  for (int i = 0; i < 7000; i++)
  {
	*ff_buffer = 0xff;
  }
  */
}


uint8_t h = 0;

void loop() {
  
  // uint32_t t_dma = micros();
  // Serial.print("dma start");
  adc_dma(adcbuf, HWORDS);
  while (!dmadone) // await DMA done isr
  {
    /*if(micros() - t_dma > 400000) // dma failed -> reset it
    {
		adc_dma(adcbuf, HWORDS);	
    }*/
  }/*
  uint32_t t = micros() - t_dma;
  Serial.print(" dma done ");
  Serial.println(t);
  delay(1);
  /*
  std::array<double, HWORDS> samplesArray = {0};
  std::array<double, HWORDS> complexArray = {0};
  for (int i = 0; i < HWORDS; i++)
  {
    samplesArray[i] = (3.3 / 1024.0 * adcbuf[i]) - (3.3/2); // scale to 10bit resolution (2^10 = 1024)
    complexArray[i] = 0;
  }
  
  /*for (int i = 0; i < HWORDS; i++)
  {
    Serial.print("0 4096 ");
    Serial.println(adcbuf[i]);
  }*/
  /*
  fft.Windowing(samplesArray.data(),samplesArray.size(), FFT_WIN_TYP_HAMMING, FFT_FORWARD);  
  fft.Compute(samplesArray.data(), complexArray.data(), samplesArray.size(), FFT_FORWARD);
  fft.ComplexToMagnitude(samplesArray.data(), complexArray.data(), samplesArray.size());
  // in samples stehen jetzt die fft werte (index 0 bis samples->size()/2)
  double peak = fft.MajorPeak(samplesArray.data(), samplesArray.size(), 15625);
  Serial.print(peak); Serial.println(" Hz");
  
  // print out
  for(int i = 0; i < 30; i++)
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
  h += 1;
  */
}

