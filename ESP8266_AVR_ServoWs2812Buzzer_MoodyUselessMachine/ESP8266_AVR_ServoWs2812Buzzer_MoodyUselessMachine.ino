#include <EEPROM.h>

/*
  Switch on ESP8266 e.g. ESP-01
  - GPIO0 (D3 on NodeMCU) & GPIO2 (D4 on NodeMCU) cannot be used for connecting the switch (they are used during the boot process)
  - GPIO3 (RX) and GPIO1 (TX) can be used (e.g. on ESP-01), but make sure the Serial port is not used by undefining DEBUG_SERIAL. RX is preferred over TX as its use as Serial port is also input
  - GPIO5 (D1 on NodeMCU) & GPIO4 (D2  on NodeMCU) are possible if I2C is not being used on these pins
  - GPIO14 (D5 on NodeMCU), GPIO12 (D6 on NodeMCU) and GPIO13 (D7 on NodeMCU) are the preferred switch inputs
  - GPIO15 (D8 on NodeMCU) doesn't support INPUT_PULLUP and is being used during boot. Don't use it for the switch. It would need pinmode INPUT as it typically has an external pulldown resistor
  - GPIO16 (D0 on NodeMCU) is not possible in this code as it would need INPUT_PULLDOWN_16 and switch to be connected to VCC
  - GPIO10 should also be possible (not tested)

  Servo on ESP8266 e.g. ESP-01
  - On NodeMCU, the servo can be connected to any of the following pwm pins: D1 - D8:
      - GPIO5 (D1), GPIO4 (D2) are possible if I2C is not being used on these pins
      - GPIO14 (D5), GPIO12 (D6), GPIO13 (D7) are the preferred servo pins
      - GPIO15 (D8) ?? to be tested as it is being used during boot (the servo acts as pulldown resistor, should be ok but to be confirmed)
      - GPIO0 (D3) and GPIO2 (D4) are also possible. But as they are used during the boot process, an external pull-up resistor is necessary
        (e.g. 220 kOhm connected to VCC)
  - GPIO3 (RX) and GPIO1 (TX) maybe also can be used (e.g. on ESP-01) - to be tested, but make sure the Serial port is not used by undefining DEBUG_SERIAL. RX is preferred over TX as its use as Serial port is also input
  - Maybe also GPIO10 works (to be tested)
  - GPIO16 (D0 on NodeMCU) cannot be used (is not a PWM pin)

*/

// #define HWMODE_ESP01

#ifdef __AVR_ATtiny85__
// PB0 & PB1 are PWM channels. PB0 is the builtin LED on the USB programmer
// PB0 & PB2 are used during upload (don't use it for switch/servo)
#define LED_PWM1_PIN PB0
#define LED_PWM2_PIN PB1
#define SERVO_PIN    PB3
#define SWITCH_PIN   PB4

#else // __AVR_ATtiny85__

#ifdef HWMODE_ESP01 // ESP-01
#include <ESP8266WiFi.h>
#define SERVO_PIN      0 // GPIO0, needs external pull-up resistor
#define LED_WS2812_PIN 1 // GPIO1, TX port
#define BUZZER_PIN     2 // GPIO2
#define SWITCH_PIN     3 // GPIO3, RX port

#else // ifdef HWMODE_ESP01

#ifdef ESP8266 // NodeMCU

#include <ESP8266WiFi.h>
#define SERVO_PIN    12        // GPIO12 (D6 on NodeMCU)
#define LED_WS2812_PIN 0       // GPIO0  (D3 on NodeMCU)
#define BUZZER_PIN 5           // GPIO5  (D1 on NodeMCU)
#define SWITCH_PIN 14          // GPIO14 (D5 on NodeMCU)
#define LED_PIN LED_BUILTIN    // GPIO16, D0 on NodeMCU

#else // ifdef ESP8266 

// AVR
#define SERVO_PIN 6
#define LED_WS2812_PIN 5
#define BUZZER_PIN 8
#define SWITCH_PIN 2
#define LED_PIN 13

#endif // ifdef ESP8266 

#define DEBUG_SERIAL Serial

#endif // ifdef HWMODE_ESP01
#endif // __AVR_ATtiny85__

// -------------------------------
// Buzzer definitions

#ifdef BUZZER_PIN
#include "rtttl.h"

/*
  Some sources of RTTTL tunes:
  http://www.picaxe.com/RTTTL-Ringtones-for-Tune-Command/
  https://github.com/KohaSuomi/emb-rtttl/tree/master/rtttl
  http://mines.lumpylumpy.com/Electronics/Computers/Software/Cpp/MFC/RingTones.RTTTL
*/

#define NUMSONGS 20
const char song_P1[] PROGMEM = "A-Team:d=8,o=5,b=125:4d#6,a#,2d#6,16p,g#,4a#,4d#.,p,16g,16a#,d#6,a#,f6,2d#6,16p,c#.6,16c6,16a#,g#.,2a#";
const char song_P2[] PROGMEM = "Flntstn:d=4,o=5,b=200:g#,c#,8p,c#6,8a#,g#,c#,8p,g#,8f#,8f,8f,8f#,8g#,c#,d#,2f,2p,g#,c#,8p,c#6,8a#,g#,c#,8p,g#,8f#,8f,8f,8f#,8g#,c#,d#,2c#";
const char song_P3[] PROGMEM = "DonkeyKong:d=4,o=5,b=200:8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6";
const char song_P4[] PROGMEM = "The Final Countdown:d=4,o=5,b=125:p,8p,16b,16a,b,e,p,8p,16c6,16b,8c6,8b,a,p,8p,16c6,16b,c6,e,p,8p,16a,16g,8a,8g,8f#,8a,g.,16f#,16g,a.,16g,16a,8b,8a,8g,8f#,e,c6,2b.,16b,16c6,16b,16a,1b";
const char song_P5[] PROGMEM = "Muppets:d=4,o=5,b=250:c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,8a,8p,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,8e,8p,8e,g,2p,c6,c6,a,b,8a,b,g,p,c6,c6,a,8b,a,g.,p,e,e,g,f,8e,f,8c6,8c,8d,e,8e,d,8d,c";
const char song_P6[] PROGMEM = "Take On Me:d=8,o=5,b=160:f#,f#,f#,d,p,b4,p,e,p,e,p,e,g#,g#,a,b,a,a,a,e,p,d,p,f#,p,f#,p,f#,e,e,f#,e,f#,f#,f#,d,p,b4,p,e,p,e,p,e,g#,g#,a,b,a,a,a,e,p,d,p,f#,p,f#,p,f#,e,e5";
const char song_P7[] PROGMEM = "TopGun:d=4,o=5,b=31:32p,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,16f,d#,16c#,16g#,16g#,32f#,32f,32f#,32f,16d#,16d#,32c#,32d#,16f,32d#,32f,16f#,32f,32c#,g#";
const char song_P8[] PROGMEM = "The Simpsons:d=4,o=5,b=160:c.6,e6,f#6,8a6,g.6,e6,c6,8a,8f#,8f#,8f#,2g,8p,8p,8f#,8f#,8f#,8g,a#.,8c6,8c6,8c6,c6";
const char song_P9[] PROGMEM = "Popeye:d=8,o=6,b=160:a5,c,c,c,4a#5,a5,4c,2p,c,d,a#5,d,4f,d,2c,p,c,d,a#5,d,f,e,d,c,d,c,a5,f5,a5,c,d,c,4a#5,g5,2f5";
const char song_P10[] PROGMEM = "Mozart:d=16,o=5,b=125:16d#,c#,c,c#,8e,8p,f#,e,d#,e,8g#,8p,a,g#,g,g#,d#6,c#6,c6,c#6,d#6,c#6,c6,c#6,4e6,8c#6,8e6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8a#,4g#,d#,32c#,c,c#,8e,8p,f#,e,d#,e,8g#,8p,a,g#,g,g#,d#6,c#6,c6,c#6,d#6,c#6,c6,c#6,4e6,8c#6,8e6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8c#6,32b,32c#6,d#6,8c#6,8b,8a#,4g#";
const char song_P11[] PROGMEM = "De Smurfen:d=32,o=5,b=200:4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8f#,p,8a#,p,4g#,4p,g#,p,a#,p,b,p,c6,p,4c#6,16p,4f#6,p,16c#6,p,8d#6,p,8b,p,4g#,16p,4c#6,p,16a#,p,8b,p,8f,p,4f#";
const char song_P12[] PROGMEM = "PacMan:b=160:32b,32p,32b6,32p,32f#6,32p,32d#6,32p,32b6,32f#6,16p,16d#6,16p,32c6,32p,32c7,32p,32g6,32p,32e6,32p,32c7,32g6,16p,16e6,16p,32b,32p,32b6,32p,32f#6,32p,32d#6,32p,32b6,32f#6,16p,16d#6,16p,32d#6,32e6,32f6,32p,32f6,32f#6,32g6,32p,32g6,32g#6,32a6,32p,32b.6";
const char song_P13[] PROGMEM = "MissionImpossi:d=4,o=5,b=180:8d#6,8c6,1g,8d#6,8c6,1f#,8d#6,8c6,1f,8d#,8f,1p,8g#,8g,1f#6,8g#,8g,1f6,8g#,8g,1e6,8d#6,8d6,2p";
const char song_P14[] PROGMEM = "Star Trek:d=4,o=5,b=63:8f.,16a#,d#.6,8d6,16a#.,16g.,16c.6,f6";
const char song_P15[] PROGMEM = "KnightRider:d=4,o=5,b=63:16e,32f,32e,8b,16e6,32f6,32e6,8b,16e,32f,32e,16b,16e6,d6,8p,p,16e,32f,32e,8b,16e6,32f6,32e6,8b,16e,32f,32e,16b,16e6,f6,p";
const char song_P16[] PROGMEM = "Tetris:d=4,o=5,b=200:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a,8p,d6,8f6,a6,8g6,8f6,e6,8e6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,a";
const char song_P17[] PROGMEM = "St Wars:d=4,o=5,b=180:8f,8f,8f,2a#.,2f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8d#6,2c6,p,8f,8f,8f,2a#.,2f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8d#6,2c6";
const char song_P18[] PROGMEM = "Transformers:d=16,o=6,b=285:e7,f7,e7,d#7,4d7,4p,d,d,d,d,d,d,d,d,e,e,e,e,f,f,f,f,f,f,f,f,8a7,8a#7,8a7,8p,4d7,2p,d,d7,d,d7,d,d7,d,d7,e,e7,e,e7,f,f7,f,f7,f,f7,f,f7,a5,a5,a5,a5,a#5,a#,a#5,a#,a#5,a#,a#5,a#,a#5,a#,a#5,a#,4p,8d,8p,4e,4f,4p,4f,4p,2g,4a,4a#,4p,g,g7,g,g7,g,g7,g,g7,4e,4g,4a,4p,f,f7,f,f7,f,f7,f,f7,4e,4f,4g,4p,e,e7,e,e7,e,e7,e,e7,e,e7,e,e7,4p,4d,4c#,8e,8p,4d,2d,d,d7,d,d7,d,d7,d,d7";
const char song_P19[] PROGMEM = "Monty Python:d=8,o=5,b=180:d#6,d6,4c6,b,4a#,a,4g#,g,f,g,g#,4g,f,2a#,p,a#,g,p,g,g,f#,g,d#6,p,a#,a#,p,g,g#,p,g#,g#,p,a#,2c6,p,g#,f,p,f,f,e,f,d6,p,c6,c6,p,g#,g,p,g,g,p,g#,2a#,p,a#,g,p,g,g,f#,g,g6,p,d#6,d#6,p,a#,a,p,f6,f6,p,f6,2f6,p,d#6,4d6,f6,f6,e6,f6,4c6,f6,f6,e6,f6,a#,p,a,a#,p,a,2a#";
const char song_P20[] PROGMEM = "StarWars/Imp:d=4,o=5,b=112:8d.,16p,8d.,16p,8d.,16p,8a#4,16p,16f,8d.,16p,8a#4,16p,16f,d.,8p,8a.,16p,8a.,16p,8a.,16p,8a#,16p,16f,8c#.,16p,8a#4,16p,16f,d.,8p,8d.6,16p,8d,16p,16d,8d6,8p,8c#6,16p,16c6,16b,16a#,8b,8p,16d#,16p,8g#,8p,8g,16p,16f#,16f,16e,8f,8p,16a#4,16p,2c#";


const char * const songlist [NUMSONGS] =
{
  song_P1,
  song_P2,
  song_P3,
  song_P4,
  song_P5,
  song_P6,
  song_P7,
  song_P8,
  song_P9,
  song_P10,
  song_P11,
  song_P12,
  song_P13,
  song_P14,
  song_P15,
  song_P16,
  song_P17,
  song_P18,
  song_P19,
  song_P20,
};


ProgmemPlayer player(BUZZER_PIN);
#endif

// -------------------------------------
// Servo definitions
#ifdef __AVR_ATtiny85__
#include "Servo8Bit.h"
#define SERVO_TYPE Servo8Bit
#else
#include <Servo.h>
#define SERVO_TYPE Servo
#endif

SERVO_TYPE fingerServo;

int fingerServoPowerOff = 800; // power off internal switch
int fingerServoFrom     = 1000; // starting position, close to internal switch
int fingerServoDoorFrom = 1100;  // door closed
int fingerServoDoorMid  = 1200;  // door slightly open
int fingerServoDoorMid2 = 1350;  // door 1/2 open
int fingerServoDoorMid3 = 1450;  // door 3/4 open
int fingerServoDoorTo   = 1550;  // door maximum open
int fingerServoMid      = 1750; // a bit from the switch
int fingerServoMid2     = 1950; // close to switch
int fingerServoTo       = 2130; // move the switch

long currentslowdown = 200;

// ----------------------------------------
// WS2812 ledstrip definitions

#ifdef LED_WS2812_PIN

#ifdef ESP8266
#define FASTLED_ESP8266_RAW_PIN_ORDER
#endif
#include <FastLED.h>

// How many leds in your strip?
#define NUMLEDPIXELS      2

// Define the array of leds
CRGB leds[NUMLEDPIXELS];
#define LEDSTRIP_MAX_BRIGHTNESS 20

#define COLOR_TYPE CRGB

#define COLOR_BLACK  CRGB::Black
#define COLOR_WHITE  CRGB::White
#define COLOR_BLUE   CRGB::Blue
#define COLOR_PURPLE CRGB::Purple
#define COLOR_YELLOW CRGB::Yellow
#define COLOR_RED    CRGB::Red
#define COLOR_GREEN  CRGB::Green
#define COLOR_DARKSEAGREEN  CRGB::DarkSeaGreen


#else // ifdef LED_WS2812_PIN
#define NUMLEDPIXELS      2
#define COLOR_TYPE uint8_t

#define COLOR_BLACK  0
#define COLOR_WHITE  255
#define COLOR_BLUE   150
#define COLOR_PURPLE 150
#define COLOR_YELLOW 150
#define COLOR_RED    150
#define COLOR_GREEN  150
#define COLOR_DARKSEAGREEN 150

#endif // ifdef LED_WS2812_PIN

COLOR_TYPE currentcolor;

int currentmode;

#define LEDSTRIP_DELAY_MAX 1000
#define LEDSTRIP_DELAY_MIN 200
long currentblinktime;

#define TIMEOUT_MS 30000L // servo will go to power off position after x milliseconds of inactivity
long last_activity;

enum
{
  MODE_LED_KITT = 0,
  MODE_LED_ON,
  MODE_LED_BLINK,
  MODE_LED_OFF,
  MODE_LED_RAINBOW,
};

// EEPROM defines
// TODO: met struct werken en put & get & sizeof

#define EEPROM_SIZE 2
#define EEPROM_CONFIG_TEST    0
#define EEPROM_SEQUENCE       1

#define EEPROM_CONFIG_TEST_VALUE 0x4C

#define NUM_SEQUENCES 10

static int currentseq = -1;

int getnextseq()
{
  randomSeed(millis() % 1024);
  return (int)random(NUM_SEQUENCES );
}

void pinModeGpio(int pinnr)
{
  // ESP-01 / ESP8266 pins RX/TX as GPIO
#ifdef ESP8266
  if ((pinnr == 1) || (pinnr == 3)) // GPIO1, GPIO3 : RX & TX ESP-01
  {
    pinMode (pinnr, FUNCTION_3);
  }
#endif
}

void eeprom_reset()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("eeprom_reset");
  DEBUG_SERIAL.flush();
#endif
  EEPROM.write(EEPROM_CONFIG_TEST, EEPROM_CONFIG_TEST_VALUE);
  EEPROM.write(EEPROM_SEQUENCE, 0);
#ifdef ESP8266
  EEPROM.commit();
#endif
}

void eeprom_init()
{
#ifdef ESP8266
  EEPROM.begin(EEPROM_SIZE);
#endif
  if (EEPROM.read(EEPROM_CONFIG_TEST) == EEPROM_CONFIG_TEST_VALUE)
  {
    currentseq = EEPROM.read(EEPROM_SEQUENCE);
    if ((currentseq < 0) || (currentseq >= PLAYLIST_LENGTH))
    {
      eeprom_reset();
      currentseq = 0;
    }
    else
    {
#ifdef DEBUG_SERIAL
      DEBUG_SERIAL.print("currentseq: ");
      DEBUG_SERIAL.println(currentseq);
#endif
    }
  }
  else
  {
    eeprom_reset();
  }
}

void eeprom_write_currentseq()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("eeprom_write_currentseq");
#endif
  EEPROM.write(EEPROM_SEQUENCE, currentseq);
#ifdef ESP8266
  EEPROM.commit();
#endif
}

void loop_led_on(bool updateSelect, bool updateRGB)
{
  if (updateSelect || updateRGB)
  {
#ifdef LED_WS2812_PIN
    fill_solid (leds, NUMLEDPIXELS, currentcolor);
    FastLED.show();
#endif
#ifdef LED_PWM1_PIN
    analogWrite(LED_PWM1_PIN, currentcolor);
#endif
#ifdef LED_PWM2_PIN
    analogWrite(LED_PWM2_PIN, currentcolor);
#endif
  }
}

void loop_led_blink(bool updateSelect)
{
  static long lasttoggletime = 0;
  static int toggle = 0;
  long currenttime = millis();

  if (updateSelect || currenttime - lasttoggletime > currentblinktime)
  {
    lasttoggletime = currenttime;

    toggle = !toggle;
    if (toggle)
    {
#ifdef LED_WS2812_PIN
      fill_solid (leds, NUMLEDPIXELS, currentcolor);
#endif
#ifdef LED_PWM1_PIN
      analogWrite(LED_PWM1_PIN, currentcolor);
#endif
#ifdef LED_PWM2_PIN
      analogWrite(LED_PWM2_PIN, currentcolor);
#endif
    }
    else
    {
#ifdef LED_WS2812_PIN
      fill_solid (leds, NUMLEDPIXELS, COLOR_BLACK);
#endif
#ifdef LED_PWM1_PIN
      analogWrite(LED_PWM1_PIN, 0);
#endif
#ifdef LED_PWM2_PIN
      analogWrite(LED_PWM2_PIN, 0);
#endif
    }
#ifdef LED_WS2812_PIN
    FastLED.show();
#endif
  }
}


void loop_led_KITT(bool updateSelect)
{
  static long starttime = 0;
  long currenttime = millis();

  if (updateSelect)
  {
    starttime = currenttime;
  }

  long passedtime = (currenttime - starttime) % currentblinktime;
  float p = ((float)passedtime * (float)(2 * (NUMLEDPIXELS - 1)) / (float)currentblinktime) - (NUMLEDPIXELS - 1);
  p = (NUMLEDPIXELS - 1) - fabs(p);

  int b1 = (int)(255.0 * (p - floor(p)));
  int b2 = (int)(255.0 * (1.0 - (p - floor(p)) ));

#ifdef LED_WS2812_PIN
  FastLED.clear();

  int i = (int)floor(p);
  leds[i] = currentcolor;
  leds[i].fadeLightBy( b1 );

  i = (int)ceil(p);
  leds[i] = currentcolor;
  leds[i].fadeLightBy( b2 );

  FastLED.show();
#endif
#ifdef LED_PWM1_PIN
  analogWrite(LED_PWM1_PIN, b1);
#endif
#ifdef LED_PWM2_PIN
  analogWrite(LED_PWM2_PIN, b2);
#endif
}

void loop_led_rainbow(bool updateSelect)
{
  static long starttime = 0;
  long currenttime = millis();

  if (updateSelect)
  {
    starttime = currenttime;
  }

  long passedtime = (currenttime - starttime) % currentblinktime;
#ifdef  LED_WS2812_PIN
  int wheelpos = map(passedtime, 0, currentblinktime, 0, 255);
  fill_solid (leds, NUMLEDPIXELS, CHSV( wheelpos, 255, 255));
  FastLED.show();
#endif
#if defined(LED_PWM1_PIN) || defined (LED_PWM2_PIN)
  float p = ((float)passedtime * (float)(2) / (float)currentblinktime) - 1;
  p = 1 - fabs(p);

  int b = (int)(255.0 * (p ));
#ifdef LED_PWM1_PIN
  analogWrite(LED_PWM1_PIN, b);
#endif
#ifdef LED_PWM2_PIN
  analogWrite(LED_PWM2_PIN, b);
#endif
#endif
}

void loop_led_off(bool updateSelect)
{

  if (updateSelect)
  {
#ifdef LED_WS2812_PIN
    FastLED.clear();
    FastLED.show();
#endif
#ifdef LED_PWM1_PIN
    analogWrite(LED_PWM1_PIN, 0);
#endif
#ifdef LED_PWM2_PIN
    analogWrite(LED_PWM2_PIN, 0);
#endif
  }
}

void updateledstrip(bool updateSelect, bool updateRGB)
{
  switch (currentmode)
  {
    case MODE_LED_KITT:
      loop_led_KITT(updateSelect);
      break;

    case MODE_LED_ON:
      loop_led_on(updateSelect, updateRGB);
      break;

    case MODE_LED_BLINK:
      loop_led_blink(updateSelect);
      break;

    case MODE_LED_OFF:
      loop_led_off(updateSelect);
      break;

    case MODE_LED_RAINBOW:
      loop_led_rainbow(updateSelect);
      break;
  }
}

void ledstrip_setmode(int newmode, COLOR_TYPE newcolor)
{
  currentcolor = newcolor;
  currentmode = newmode;
  updateledstrip(true, true);
}

void ledstrip_setmode_delay(int newmode, COLOR_TYPE newcolor, int newblinktime)
{
  currentcolor = newcolor;
  currentmode = newmode;
  currentblinktime = newblinktime;
  updateledstrip(true, true);
}

// https://github.com/todbot/ServoEaser/blob/master/examples/ServoEaser3Callbacks/ServoEaser3Callbacks.ino
float ServoEaser_linear (float t, float b, float c, float d) {
  return c * t / d + b;
}

inline float ServoEaser_easeInOutCubic(float t, float b, float c, float d)
{
  if ((t /= d / 2) < 1) return c / 2 * t * t * t + b;
  return c / 2 * ((t -= 2) * t * t + 2) + b;
}

void sweep(SERVO_TYPE *srv, int from, int to, int delayus)
{
  unsigned long startMillis = millis();
  float currPos;
  unsigned long durMillis = ((unsigned long)(delayus + currentslowdown) * (unsigned long)(abs(to - from))) / 1000L;
  unsigned long currentMillis = millis();
  unsigned long endMillis = startMillis + durMillis;

  while (currentMillis < endMillis)
  {
    currentMillis = millis();
    // use ServoEaser_linear for linear movement, ServoEaser_easeInOutCubic for natural movement
    currPos = ServoEaser_easeInOutCubic( currentMillis - startMillis, (float)from, (float)(to - from), durMillis );

    srv->writeMicroseconds(currPos);
    updateledstrip(false, false);
#ifdef BUZZER_PIN
    player.pollSong();
#endif
    yield();
  }
}

void sweep_delay(unsigned long durMillis)
{
  unsigned long currentMillis = millis();
  unsigned long endMillis = currentMillis + durMillis;

#ifdef LED_WS2812_PIN
  FastLED.delay(2); // we have some free time, this will update the LED's
#endif
  while (currentMillis < endMillis)
  {
    currentMillis = millis();

#ifdef BUZZER_PIN
    player.pollSong();
#endif
    updateledstrip(false, false);
    yield();
  }
}

#ifdef BUZZER_PIN
void player_stopPlaying(bool waitUntilEndTune)
{
  if (waitUntilEndTune)
  {
    while (player.pollSong()) {
      updateledstrip(false, false);
      yield();
    }
  }
  else
  {
    player.silence();
  }
  player.setSong(NULL);
}
#endif

// The sequences are inspired by the "Moody Useless Machine", Lamja Electronics
// http://www.lamja.com/?p=451
// http://www.lamja.com/blogfiles/UselessMachine.pde

void sequence1()
{
  currentslowdown = 200;
  ledstrip_setmode(MODE_LED_ON, COLOR_BLUE );
  sweep_delay(700);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 3000);
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 500);
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoMid, 1800);
  sweep(&fingerServo, fingerServoMid, fingerServoTo, 500);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 500);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence2()
{
  currentslowdown = 200;
  sweep_delay(800);
  ledstrip_setmode_delay(MODE_LED_BLINK, COLOR_WHITE, 200 );
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid2, 3000);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 3000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 3000);
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoMid, 1800);
  sweep(&fingerServo, fingerServoMid, fingerServoTo, 500);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 500);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence3()
{
  currentslowdown = 200;
  ledstrip_setmode_delay(MODE_LED_KITT, COLOR_GREEN, 600 );
  sweep_delay(50);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence4()
{
  currentslowdown = 200;
  ledstrip_setmode(MODE_LED_ON, COLOR_PURPLE );
  sweep_delay(500);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid2, 1);
  sweep_delay(450);
  ledstrip_setmode_delay(MODE_LED_RAINBOW, 0, 5000 );
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 30000);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence5()
{
  currentslowdown = 200;

  ledstrip_setmode(MODE_LED_ON, COLOR_YELLOW );
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);

  ledstrip_setmode_delay(MODE_LED_KITT, COLOR_YELLOW, 200 );

  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  sweep_delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  sweep_delay(110);
  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  sweep_delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  sweep_delay(110);
  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  sweep_delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  sweep_delay(110);

  ledstrip_setmode(MODE_LED_ON, COLOR_YELLOW );

  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence6()
{
  currentslowdown = 400;

  ledstrip_setmode(MODE_LED_ON, COLOR_PURPLE );
  sweep_delay(1500);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 1);
  sweep_delay(450);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1000);

  sweep_delay(2000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);

  ledstrip_setmode_delay(MODE_LED_KITT, COLOR_WHITE, 1000 );
  sweep_delay(2000);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );

  sweep(&fingerServo, fingerServoDoorTo, fingerServoFrom, 1);
  sweep_delay(200);
}

void sequence7()
{
  currentslowdown = 300;

  ledstrip_setmode_delay(MODE_LED_RAINBOW, 0, 1000 );

  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);

  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence8()
{
  currentslowdown = 300;

  ledstrip_setmode(MODE_LED_ON, COLOR_BLUE );
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);

  ledstrip_setmode_delay(MODE_LED_BLINK, COLOR_BLUE, 100 );
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);


  ledstrip_setmode(MODE_LED_ON, COLOR_BLUE );
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence9()
{
  currentslowdown = 200;

  sweep_delay(1000);

  ledstrip_setmode(MODE_LED_ON, COLOR_RED );

  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 2000);
  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1000);

  ledstrip_setmode_delay(MODE_LED_BLINK, COLOR_RED, 100 );

  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  sweep_delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);

  ledstrip_setmode(MODE_LED_ON, COLOR_RED );

  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 5000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void sequence10()
{
  char songname[20 + 1];

  currentslowdown = 200;
#ifdef BUZZER_PIN
  int songnr = random(NUMSONGS);
  player.setSong(songlist[songnr]); // or take one song e.g. song_P1
  player.getName(songname, 20);
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.print("Start song nr ");
  DEBUG_SERIAL.println(songnr);
  DEBUG_SERIAL.print("Name: ");
  DEBUG_SERIAL.println(songname);
#endif
#endif
  ledstrip_setmode_delay(MODE_LED_RAINBOW, 0, 10000 );
  sweep_delay(800);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 30000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 3000);
#ifdef BUZZER_PIN
  player_stopPlaying(false);
#endif
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 3000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  sweep_delay(300);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );
}

void playsequence()
{
  void (*sequences[NUM_SEQUENCES])() = {
    sequence1,
    sequence2,
    sequence3,
    sequence4,
    sequence5,
    sequence6,
    sequence7,
    sequence8,
    sequence9,
    sequence10,
  };

#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.print("Starting sequence ");
  DEBUG_SERIAL.println(currentseq);
  DEBUG_SERIAL.flush();
#endif

  if ((currentseq >= 0) && (currentseq < NUM_SEQUENCES ))
  {
    sequences[currentseq]();
  }

#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("Sequence ended");
  DEBUG_SERIAL.flush();
#endif
}

void setup() {
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("Setup Moody useless machine");
  DEBUG_SERIAL.flush();
#endif

#ifdef ESP8266
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
#endif

  eeprom_init();

#ifdef SWITCH_PIN
  pinModeGpio(SWITCH_PIN);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
#endif

#ifdef LED_PIN
  pinModeGpio(LED_PIN);
  pinMode(LED_PIN, OUTPUT);
#endif

#ifdef BUZZER_PIN
  pinModeGpio(BUZZER_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  player.transpose(-2);
#endif

  pinModeGpio(SERVO_PIN);
  fingerServo.writeMicroseconds(fingerServoFrom);
  fingerServo.attach(SERVO_PIN);

#ifdef LED_PWM1_PIN
  pinModeGpio(LED_PWM1_PIN);
  pinMode(LED_PWM1_PIN, OUTPUT);
#endif

#ifdef LED_PWM2_PIN
  pinModeGpio(LED_PWM2_PIN);
  pinMode(LED_PWM2_PIN, OUTPUT);
#endif

#ifdef LED_WS2812_PIN
  pinModeGpio(LED_WS2812_PIN);
  FastLED.addLeds<NEOPIXEL, LED_WS2812_PIN>(leds, NUMLEDPIXELS);
  FastLED.setBrightness(LEDSTRIP_MAX_BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  FastLED.delay(2);
#endif

  currentcolor = COLOR_BLACK;
  currentmode = MODE_LED_OFF;
  currentblinktime = LEDSTRIP_DELAY_MAX;

  last_activity = millis();
}


void lookAroundPowerDown()
{
  // "Look around" to see if it's safe to power power down the box
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);

  ledstrip_setmode_delay(MODE_LED_KITT, COLOR_DARKSEAGREEN, 1000 );
  sweep_delay(2000);
  ledstrip_setmode(MODE_LED_OFF, COLOR_BLACK );

  sweep(&fingerServo, fingerServoDoorTo, fingerServoFrom, 1);
  sweep_delay(200);

  // Move servo to Power off position
  fingerServo.writeMicroseconds(fingerServoPowerOff);
  delay(100);

  // this code is normally no longer executed, as the power should be shut down by the servo.
  // in case the power is still on, just continue

  last_activity = millis();
}


void loop() {
  if (millis() > last_activity + TIMEOUT_MS)
  {
    lookAroundPowerDown();
  }

#ifdef SWITCH_PIN
  if (digitalRead(SWITCH_PIN) == LOW) {
#ifdef LED_PIN
    digitalWrite(LED_PIN, LOW);
#endif
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("Switch on");
    DEBUG_SERIAL.flush();
#endif
  }
  else {
#ifdef LED_PIN
    digitalWrite(LED_PIN, HIGH);
#endif
    delay(100);
    return;
  }
#endif

  currentseq = getnextseq();

  eeprom_write_currentseq();

  playsequence();

  last_activity = millis();
}
