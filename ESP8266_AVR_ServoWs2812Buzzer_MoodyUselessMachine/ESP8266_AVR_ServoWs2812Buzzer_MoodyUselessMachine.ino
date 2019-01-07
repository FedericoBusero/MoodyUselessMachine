#include <Servo.h>
#include <EEPROM.h>

#ifdef ESP8266
#define FASTLED_ESP8266_RAW_PIN_ORDER
#endif
#include <FastLED.h>

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

#ifdef HWMODE_ESP01 // ESP-01
// TODO: check if another allocation is possible without external resistor
#include <ESP8266WiFi.h>
const int switchPin      = 3;  // GPIO3, RX port
const int fingerServoPin = 0;  // GPIO0, needs external pull-up resistor
const int ledstripPin    = 1;  // GPIO1, TX port
// #define BUZZER_PIN 2        // GPIO2 ?? needs external pull-up resistor?

#else
#ifdef ESP8266 // NodeMCU
#include <ESP8266WiFi.h>
const int switchPin      = 14; // GPIO14 (D5 on NodeMCU)
const int fingerServoPin = 12; // GPIO12 (D6 on NodeMCU)
const int ledstripPin    = 0;  // GPIO0  (D3 on NodeMCU)
// #define BUZZER_PIN 13          // GPIO13 (D7 on NodeMCU)
#define LED_PIN LED_BUILTIN    // GPIO16, D0 on NodeMCU

#else // AVR
const int switchPin      = 2;
const int fingerServoPin = 6;
const int ledstripPin    = 5;
#define BUZZER_PIN 8
#define LED_PIN 13

#endif

#define DEBUG_SERIAL Serial

#endif

#ifdef BUZZER_PIN
#include "rtttl.h"

// const char song_P[] PROGMEM = "Ghostbusters:d=4,o=5,b=180:4c5,4c5,8e5,8f5,8g5,8p, 4a#5,4a#5,4f5,4f5,4c5,4c5,8e5,8f5,8g5,8p,4a#5,4a#5,4f5";
// const char song_P[] PROGMEM = "dkong_level:d=4,o=5,b=200:c6,32p,8d6,8p,f6,8p,8d6,16p,8c6,16p,8d6,16p,a#";
// const char song_P[] PROGMEM = "DonkeyKong:d=4,o=5,b=200:8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6,8a#,8p,8d6,16p,16f.6,16g.6,16f.6";
// const char song_P[] PROGMEM = "Dambuste:d=4,o=5,b=63:4f6,8a#6,8f6,8f6,16d#6,16d6,8d#6,8f6,4d6,8f6,8d6,8d6,16c6,16a#,8a,8c6,8a#.,16c6,8d6,8g6,8f.6,16d6,4f6,8c6,8f6,16g6,16a6,8a#6,4a6,4p";
// const char song_P[] PROGMEM = "AxelF:d=4,o=5,b=125:32p,8g,8p,16a#.,8p,16g,16p,16g,8c6, 8g,8f,8g,8p,16d.6,8p,16g,16p,16g,8d#6,8d6,8a#,8g,8d6,8g6, 16g,16f,16p,16f,8d,8a#,2g,p,SS,16f6,8d6,8c6,8a#,g,8a#.,16g, 16p,16g,8c6,8g,8f,g,8d.6,16g,16p,16g,8d#6,8d6,8a#,8g,8d6, 8g6,16g,16f,16p,16f,8d,8a#,2g";
// const char song_P[] PROGMEM = "PacMan:b=160:32b,32p,32b6,32p,32f#6,32p,32d#6,32p,32b6,32f#6,16p,16d#6,16p,32c6,32p,32c7,32p,32g6,32p,32e6,32p,32c7,32g6,16p,16e6,16p,32b,32p,32b6,32p,32f#6,32p,32d#6,32p,32b6,32f#6,16p,16d#6,16p,32d#6,32e6,32f6,32p,32f6,32f#6,32g6,32p,32g6,32g#6,32a6,32p,32b.6";
// const char song_P[] PROGMEM = "The Final Countdown:d=4,o=5,b=125:p,8p,16b,16a,b,e,p,8p,16c6,16b,8c6,8b,a,p,8p,16c6,16b,c6,e,p,8p,16a,16g,8a,8g,8f#,8a,g.,16f#,16g,a.,16g,16a,8b,8a,8g,8f#,e,c6,2b.,16b,16c6,16b,16a,1b";
// const char song_P[] PROGMEM = "MissionImpossi:d=4,o=5,b=180:8d#6,8c6,1g,8d#6,8c6,1f#,8d#6,8c6,1f,8d#,8f,1p,8g#,8g,1f#6,8g#,8g,1f6,8g#,8g,1e6,8d#6,8d6,2p";
// const char song_P[] PROGMEM = "Star Trek:d=4,o=5,b=63:8f.,16a#,d#.6,8d6,16a#.,16g.,16c.6,f6";
// const char song_P[] PROGMEM = "St Wars:d=4,o=5,b=180:8f,8f,8f,2a#.,2f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8d#6,2c6,p,8f,8f,8f,2a#.,2f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8c6,2a#.6,f.6,8d#6,8d6,8d#6,2c6";
// const char song_P[] PROGMEM = "KnightRider:d=4,o=5,b=63:16e,32f,32e,8b,16e6,32f6,32e6,8b,16e,32f,32e,16b,16e6,d6,8p,p,16e,32f,32e,8b,16e6,32f6,32e6,8b,16e,32f,32e,16b,16e6,f6,p";
// const char song_P[] PROGMEM = "Flntstn:d=4,o=5,b=200:g#,c#,8p,c#6,8a#,g#,c#,8p,g#,8f#,8f,8f,8f#,8g#,c#,d#,2f,2p,g#,c#,8p,c#6,8a#,g#,c#,8p,g#,8f#,8f,8f,8f#,8g#,c#,d#,2c#";
// const char song_P[] PROGMEM = "Beethoven:d=4,o=5,b=160:c,e,c,g,c,c6,8b,8a,8g,8a,8g,8f,8e,8f,8e,8d,c,e,g,e,c6,g";
// const char song_P[] PROGMEM = "Tetris:d=4,o=5,b=200:e6,8b,8c6,8d6,16e6,16d6,8c6,8b,a,8a,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,2a,8p,d6,8f6,a6,8g6,8f6,e6,8e6,8c6,e6,8d6,8c6,b,8b,8c6,d6,e6,c6,a,a";
const char song_P[] PROGMEM = "StarWars/Imp:d=4,o=5,b=112:8d.,16p,8d.,16p,8d.,16p,8a#4,16p,16f,8d.,16p,8a#4,16p,16f,d.,8p,8a.,16p,8a.,16p,8a.,16p,8a#,16p,16f,8c#.,16p,8a#4,16p,16f,d.,8p,8d.6,16p,8d,16p,16d,8d6,8p,8c#6,16p,16c6,16b,16a#,8b,8p,16d#,16p,8g#,8p,8g,16p,16f#,16f,16e,8f,8p,16a#4,16p,2c#";

ProgmemPlayer player(BUZZER_PIN);
#endif

Servo fingerServo;

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

// WS2812 ledstrip

// How many leds in your strip?
#define NUMLEDPIXELS      2

// Define the array of leds
CRGB leds[NUMLEDPIXELS];
CRGB currentcolor;
int currentmode;

#define LEDSTRIP_DELAY_MAX 1000
#define LEDSTRIP_DELAY_MIN 200
#define LEDSTRIP_MAX_BRIGHTNESS 20
long currentblinktime;

enum
{
  MODE_LED_KITT=0,
  MODE_LED_ON,
  MODE_LED_BLINK,
  MODE_LED_OFF,
  MODE_LED_RAINBOW,
};

// EEPROM defines
// TODO: met struct werken en put & get & sizeof

#define EEPROM_SIZE 4
#define EEPROM_CONFIG_FLAG    0
#define EEPROM_CONFIG_TEST    1
#define EEPROM_SEQUENCE       2

#define EEPROM_CONFIG_TEST_VALUE 0x4B

#define SEQUENCE_START 0
#define SEQUENCE_END   17

static int currentseq = SEQUENCE_START;

int getnextseq()
{
  if (currentseq < SEQUENCE_END)
  {
    return (currentseq + 1);
  }
  else
  {
    return SEQUENCE_START;
  }
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
  EEPROM.write(EEPROM_CONFIG_FLAG, 1);
  EEPROM.write(EEPROM_CONFIG_TEST, EEPROM_CONFIG_TEST_VALUE);
  EEPROM.write(EEPROM_SEQUENCE, SEQUENCE_START);
#ifdef ESP8266
  EEPROM.commit();
#endif
}

void eeprom_init()
{
#ifdef ESP8266
  EEPROM.begin(EEPROM_SIZE);
#endif
  if (EEPROM.read(EEPROM_CONFIG_FLAG) == 1)
  {
    if (EEPROM.read(EEPROM_CONFIG_TEST) == EEPROM_CONFIG_TEST_VALUE)
    {
      currentseq = EEPROM.read(EEPROM_SEQUENCE);
      if ((currentseq < SEQUENCE_START) || (currentseq > SEQUENCE_END))
      {
        eeprom_reset();
        currentseq = SEQUENCE_START;
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
  else
  {
    eeprom_reset();
  }
}

void eeprom_write_next_sequence()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("eeprom_write_next_sequence");
#endif
  EEPROM.write(EEPROM_SEQUENCE, getnextseq());
#ifdef ESP8266
  EEPROM.commit();
#endif
}

void loop_led_on(bool updateSelect, bool updateRGB)
{
  if (updateSelect || updateRGB)
  {
    fill_solid (leds, NUMLEDPIXELS, currentcolor);
    FastLED.show();
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
      fill_solid (leds, NUMLEDPIXELS, currentcolor);
    }
    else
    {
      fill_solid (leds, NUMLEDPIXELS, CRGB::Black);
    }
    FastLED.show();
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

  FastLED.clear();
  
  int b = (int)(255.0 * (p - floor(p)));
  int i = (int)floor(p);
  leds[i] = currentcolor;
  leds[i].fadeLightBy( b );
  
  b = (int)(255.0 * (1.0 - (p - floor(p)) ));
  i = (int)ceil(p);
  leds[i] = currentcolor;
  leds[i].fadeLightBy( b );
  
  FastLED.show();
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
  int wheelpos = map(passedtime, 0, currentblinktime, 0, 255);
  fill_solid (leds, NUMLEDPIXELS, CHSV( wheelpos, 255, 255));
  FastLED.show();
}

void loop_led_off(bool updateSelect)
{

  if (updateSelect)
  {
    FastLED.clear();
    FastLED.show();
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

void ledstrip_setmode(int newmode, CRGB newcolor)
{
  currentcolor = newcolor;
  currentmode = newmode;
  updateledstrip(true, true);
}

void ledstrip_setmode_delay(int newmode, CRGB newcolor, int newblinktime)
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

void sweep(Servo *srv, int from, int to, int delayus)
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

  FastLED.delay(2); // we have some free time, this will update the LED's
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
void player_finishSong()
{
  while (player.pollSong()) {
    updateledstrip(false, false);
    yield();
  }
}
#endif

// The sequences are inspired by the "Moody Useless Machine", Lamja Electronics
// http://www.lamja.com/?p=451
// http://www.lamja.com/blogfiles/UselessMachine.pde

void sequence1()
{
  currentslowdown = 200;
  ledstrip_setmode(MODE_LED_ON, CRGB::Blue );
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
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence2()
{
  currentslowdown = 200;
  sweep_delay(800);
  ledstrip_setmode_delay(MODE_LED_BLINK, CRGB::White, 200 );
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
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence3()
{
  currentslowdown = 200;
  ledstrip_setmode_delay(MODE_LED_KITT, CRGB::Green, 600 );
  sweep_delay(50);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence4()
{
  currentslowdown = 200;
  ledstrip_setmode(MODE_LED_ON, CRGB::Purple );
  sweep_delay(500);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid2, 1);
  sweep_delay(450);
  ledstrip_setmode_delay(MODE_LED_RAINBOW, 0, 5000 );
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 30000);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence5()
{
  currentslowdown = 200;

  ledstrip_setmode(MODE_LED_ON, CRGB::Yellow );
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);

  ledstrip_setmode_delay(MODE_LED_KITT, CRGB::Yellow, 200 );

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

  ledstrip_setmode(MODE_LED_ON, CRGB::Yellow );

  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence6()
{
  currentslowdown = 400;

  ledstrip_setmode(MODE_LED_ON, CRGB::Purple );
  sweep_delay(1500);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 1);
  sweep_delay(450);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1000);

  sweep_delay(2000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);

  ledstrip_setmode_delay(MODE_LED_KITT, CRGB::White, 1000 );
  sweep_delay(2000);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );

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

  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence8()
{
  currentslowdown = 300;

  ledstrip_setmode(MODE_LED_ON, CRGB::Blue );
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);

  ledstrip_setmode_delay(MODE_LED_BLINK, CRGB::Blue, 100 );
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


  ledstrip_setmode(MODE_LED_ON, CRGB::Blue );
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence9()
{
  currentslowdown = 200;

  sweep_delay(1000);

  ledstrip_setmode(MODE_LED_ON, CRGB::Red );

  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 2000);
  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1000);

  ledstrip_setmode_delay(MODE_LED_BLINK, CRGB::Red, 100 );

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

  ledstrip_setmode(MODE_LED_ON, CRGB::Red );

  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 5000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void sequence10()
{
  currentslowdown = 200;
#ifdef BUZZER_PIN
  player.setSong(song_P);
#endif
  ledstrip_setmode_delay(MODE_LED_RAINBOW, 0, 10000 );
  sweep_delay(800);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 30000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 3000);
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 3000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorMid, 30000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 1);
  sweep_delay(300);
#ifdef BUZZER_PIN
  player_finishSong();
#endif
  ledstrip_setmode(MODE_LED_OFF, CRGB::Black );
}

void playsequence()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.print("Starting sequence ");
  DEBUG_SERIAL.println(currentseq);
  DEBUG_SERIAL.flush();
#endif

  if (currentseq == 0) sequence3();
  if (currentseq == 1) sequence1();
  if (currentseq == 2) sequence3();
  if (currentseq == 3) sequence9();
  if (currentseq == 4) sequence3();
  if (currentseq == 5) sequence5();
  if (currentseq == 6) sequence3();
  if (currentseq == 7) sequence7();
  if (currentseq == 8) sequence3();
  if (currentseq == 9) sequence2();
  if (currentseq == 10) sequence3();
  if (currentseq == 11) sequence10();
  if (currentseq == 12) sequence3();
  if (currentseq == 13) sequence8();
  if (currentseq == 14) sequence3();
  if (currentseq == 15) sequence6();
  if (currentseq == 16) sequence3();
  if (currentseq == 17) sequence4();

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

  pinModeGpio(switchPin);
  pinMode(switchPin, INPUT_PULLUP);

#ifdef LED_PIN
  pinModeGpio(LED_PIN);
  pinMode(LED_PIN, OUTPUT);
#endif

  pinModeGpio(fingerServoPin);
  fingerServo.writeMicroseconds(fingerServoFrom);
  fingerServo.attach(fingerServoPin);
  
  pinModeGpio(ledstripPin);
  FastLED.addLeds<NEOPIXEL, ledstripPin>(leds, NUMLEDPIXELS);
  FastLED.setBrightness(LEDSTRIP_MAX_BRIGHTNESS);
  FastLED.clear();

  currentcolor = CRGB::Black;
  currentmode = MODE_LED_OFF;
  currentblinktime = LEDSTRIP_DELAY_MAX;
}

void loop() {
  int next_sequence;

  if (digitalRead(switchPin) == LOW) {
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

  eeprom_write_next_sequence();

  playsequence();

  currentseq = getnextseq();

  // Power off
  fingerServo.writeMicroseconds(fingerServoPowerOff);
}
