#include <Servo.h>
#include <EEPROM.h>

#ifdef ESP8266
#define FASTLED_ESP8266_RAW_PIN_ORDER
#endif
#include <FastLED.h>

/*
  Switch on ESP8266
  - GPIO0 (D3 on NodeMCU) & GPIO2 (D4 on NodeMCU) cannot be used for connecting the switch (they are used during the boot process)
  - GPIO3 (RX) and GPIO1 (TX) can be used (e.g. on ESP-01), but make sure the Serial port is not used by undefining DEBUG_SERIAL. RX is preferred over TX as its use as Serial port is also input
  - GPIO5 (D1 on NodeMCU) & GPIO4 (D2  on NodeMCU) can be used if I2C is not being used
  - GPIO14 (D5 on NodeMCU), GPIO12 (D6 on NodeMCU) and GPIO13 (D7 on NodeMCU) are the preferred switch inputs
  - GPIO15 (D8 on NodeMCU) doesn't support INPUT_PULLUP and is being used during boot. Don't use it for the switch.
*/

// #define HWMODE_ESP01

#ifdef HWMODE_ESP01 // ESP-01
// TODO: test
#include <ESP8266WiFi.h>
const int switchPin      = 3;  // GPIO3, RX port
const int fingerServoPin = 0;  // GPIO0
const int ledstripPin    = 1;  // GPIO1, TX port

#else
#ifdef ESP8266 // NodeMCU
#include <ESP8266WiFi.h>
const int switchPin      = 14; // GPIO14 (D5 on NodeMCU)
const int fingerServoPin = 12; // GPIO12 (D6 on NodeMCU)
const int ledstripPin    = 0;  // GPIO0  (D3 on NodeMCU)
#define LED_PIN LED_BUILTIN    // GPIO16, D0 on NodeMCU

#else // AVR
const int switchPin      = 2;
const int fingerServoPin = 6;
const int ledstripPin    = 5;
#define LED_PIN 13

#endif

#define DEBUG_SERIAL Serial

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

#define SLOWDOWNMICROSEC 200 // 200 fast, 500 normal, 1000 slow

// WS2812 ledstrip

// How many leds in your strip?
#define NUMLEDPIXELS      8

// Define the array of leds
CRGB leds[NUMLEDPIXELS];
CRGB currentcolor;
int currentmode;

#define LEDSTRIP_DELAY_MAX 1000
#define LEDSTRIP_DELAY_MIN 200
long currentblinktime;

enum
{
  MODE_LED_KITT=0,
  MODE_LED_ON,
  MODE_LED_BLINK,
  MODE_LED_OFF,
  MODE_LED_KITT2,
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


// https://github.com/todbot/ServoEaser/blob/master/examples/ServoEaser3Callbacks/ServoEaser3Callbacks.ino
float ServoEaser_linear (float t, float b, float c, float d) {
  return c * t / d + b;
}

inline float ServoEaser_easeInOutCubic(float t, float b, float c, float d)
{
  if ((t /= d / 2) < 1) return c / 2 * t * t * t + b;
  return c / 2 * ((t -= 2) * t * t + 2) + b;
}

void loop_led_on(bool updateSelect, bool updateRGB)
{
  if (updateSelect || updateRGB)
  {
    for (int i = 0; i < NUMLEDPIXELS; i++) {
      leds[i] = currentcolor;
    }
    FastLED.show();
  }
}

void loop_led_blink(bool updateSelect)
{
  long blinktime = currentblinktime;
  static long lasttoggletime = 0;
  static int toggle = 0;
  long currenttime = millis();

  if (updateSelect || currenttime - lasttoggletime > blinktime)
  {
    lasttoggletime = currenttime;

    toggle = !toggle;
    if (toggle)
    {
      for (int i = 0; i < NUMLEDPIXELS; i++)
      {
        leds[i] = currentcolor;
      }
    }
    else
    {
      for (int i = 0; i < NUMLEDPIXELS; i++)
      {
        leds[i] = CRGB::Black;
      }
    }
    FastLED.show();
  }
}


void loop_led_KITT(bool updateSelect)
{
  long blinktime = currentblinktime;
  static long lastupdatetime = 0;
  static int toggle = 0;
  long currenttime = millis();
  static int currentpos = 0;
  static int dir_kitt = 0;

  if (updateSelect || (currenttime - lastupdatetime > blinktime / NUMLEDPIXELS))
  {
    lastupdatetime = currenttime;

    if (dir_kitt)
    {
      ++currentpos;
    }
    else
    {
      --currentpos;
    }
    if (currentpos >= NUMLEDPIXELS)
    {
      currentpos = NUMLEDPIXELS - 2;
      dir_kitt = 0;
    }
    if (currentpos < 0)
    {
      currentpos = 1;
      dir_kitt = 1;
    }
    leds[currentpos] = currentcolor;
    FastLED.show();
    leds[currentpos] = CRGB::Black;
  }
}


void loop_led_KITT2(bool updateSelect)
{
  long blinktime = currentblinktime;
  static long lastupdatetime = 0;
  static int toggle = 0;
  long currenttime = millis();
  static int currentpos = 0;
  static int dir_kitt = 0;

  if (updateSelect || (currenttime - lastupdatetime > blinktime / NUMLEDPIXELS))
  {
    lastupdatetime = currenttime;

    if (dir_kitt)
    {
      ++currentpos;
    }
    else
    {
      --currentpos;
    }
    if (currentpos >= (NUMLEDPIXELS + 1) / 2)
    {
      currentpos = (NUMLEDPIXELS + 1) / 2 - 2;
      dir_kitt = 0;
    }
    if (currentpos < 0)
    {
      currentpos = 1;
      dir_kitt = 1;
    }
    leds[currentpos] = currentcolor;
    leds[NUMLEDPIXELS - currentpos - 1] = currentcolor;
    FastLED.show();
    leds[currentpos] = CRGB::Black;
    leds[NUMLEDPIXELS - currentpos - 1] = CRGB::Black;
  }
}


void loop_led_off(bool updateSelect)
{

  if (updateSelect)
  {
    for (int i = 0; i < NUMLEDPIXELS; i++) {
      leds[i] = CRGB::Black;
    }
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

    case MODE_LED_KITT2:
      loop_led_KITT2(updateSelect);
      break;
  }
}


void sweep(Servo *srv, int from, int to, int delayus)
{
  unsigned long startMillis = millis();
  float currPos;
  unsigned long durMillis = ((unsigned long)(delayus + SLOWDOWNMICROSEC) * (unsigned long)(abs(to - from))) / 1000L;
  unsigned long currentMillis = millis();
  unsigned long endMillis = startMillis + durMillis;

  while (currentMillis < endMillis)
  {
    currentMillis = millis();
    // use ServoEaser_linear for linear movement, ServoEaser_easeInOutCubic for natural movement
    currPos = ServoEaser_easeInOutCubic( currentMillis - startMillis, (float)from, (float)(to - from), durMillis );

    srv->writeMicroseconds(currPos);
    updateledstrip(false, false);
    yield();
  }
}

void ledstrip_setmode(int newmode, int r, int g, int b)
{
  currentcolor = CRGB(r, g, b);
  currentmode = newmode;
  updateledstrip(true, true);
}

void ledstrip_setmode_delay(int newmode, int r, int g, int b, int newblinktime)
{
  currentcolor = CRGB(r, g, b);
  currentmode = newmode;
  currentblinktime = newblinktime;
  updateledstrip(true, true);
}

void sweep_delay(unsigned long durMillis)
{
  unsigned long currentMillis = millis();
  unsigned long endMillis = currentMillis + durMillis;

  while (currentMillis < endMillis)
  {
    currentMillis = millis();

    updateledstrip(false, false);
    yield();
  }
}

// The sequences are based on the "Moody Useless Machine", Lamja Electronics
// http://www.lamja.com/?p=451
// http://www.lamja.com/blogfiles/UselessMachine.pde

void sequence1()
{
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
}

void sequence2()
{
  sweep_delay(800);
  ledstrip_setmode_delay(MODE_LED_BLINK, 25, 25, 25, LEDSTRIP_DELAY_MIN );
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
  ledstrip_setmode(MODE_LED_OFF, 0, 0, 0 );
}

void sequence3()
{
  ledstrip_setmode_delay(MODE_LED_KITT, 0, 25, 0, LEDSTRIP_DELAY_MAX );
  sweep_delay(50);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, 0, 0, 0 );
}

void sequence4()
{
  ledstrip_setmode(MODE_LED_ON, 25, 0, 25 );
  sweep_delay(500);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid2, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 30000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
  ledstrip_setmode(MODE_LED_OFF, 0, 0, 0 );
}

void sequence5()
{
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
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
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
}

void sequence6()
{
  sweep_delay(1500);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1000);
  sweep_delay(2000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);
  sweep_delay(2000);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoFrom, 1);
  sweep_delay(200);
}

void sequence7()
{
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
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
}

void sequence8()
{
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  sweep_delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  sweep_delay(100);
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
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 1);
  sweep_delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
}

void sequence9()
{
  sweep_delay(1000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 2000);
  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1000);
  sweep_delay(1);
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
  sweep_delay(500);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 5000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 1);
  sweep_delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  sweep_delay(400);
}

void sequence10()
{
  sweep_delay(800);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 30000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoTo, 3000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoTo, fingerServoDoorTo, 3000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorMid, 30000);
  sweep_delay(1);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 1);
  sweep_delay(300);
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

  currentcolor = CRGB(0, 0, 0);
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

