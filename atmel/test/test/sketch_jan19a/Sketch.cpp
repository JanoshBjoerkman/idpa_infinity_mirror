/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <FastLED.h>
#undef min
#undef max
#include <array>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio



// LED strip
// konstanten
constexpr static unsigned char PIN_LED_CLOCK = 9;
constexpr static unsigned char PIN_LED_DATA = 6;
constexpr static EOrder COLOR_ORDER = BGR;
constexpr static int NUM_LEDS = 60;
// led array
std::array<CRGB, NUM_LEDS> leds;

void setup() {
  // setup code
  FastLED.addLeds<DOTSTAR, PIN_LED_DATA, PIN_LED_CLOCK, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  255 );
  // keine ahnung obs auswirkung hat, meine PowerBank hat 2.1A darum hab ich mal 2A max eingestellt. Sollte die Helligkeit automatisch regeln wenn zuviele Leds zu hell eingestellt sind
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 2000);

  // alle leds gleiche farbe
  CHSV color(0, 0, 0) ; // rot
  fill_solid(leds.data(), leds.size(), color);
  
  
}

int counter = 0;
bool color_flag = true;
void loop() {
  
    for(int i = 0; i < 30; i++)
    {
        double col = 255/30 * i;
        CHSV color(static_cast<uint8_t>(col), 255, counter);
        leds[i] = color;
        leds[59-i] = color;  
    }
    FastLED.show();
    if(color_flag)
    {
      if(counter < 255)
      {
        //Serial.println(counter);
        counter++;
      }
      else
      {
        color_flag = false;
        counter--;
      } 
    }
    else
    {
      if(counter > 0)
      {
        counter--;
      }
      else
      {
        color_flag = true;
        counter++;
      } 
    }
    delay(10);
}

