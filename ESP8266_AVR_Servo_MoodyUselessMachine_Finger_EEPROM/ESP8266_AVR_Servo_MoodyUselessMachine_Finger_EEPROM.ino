// The sequences are based on the "Moody Useless Machine", Lamja Electronics
// http://www.lamja.com/?p=451
// http://www.lamja.com/blogfiles/UselessMachine.pde

#include <Servo.h>
#include <EEPROM.h>

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
const int switchPin      = 3;  // GPIO3, RX port
const int fingerServoPin = 0;  // GPIO0

#else
#ifdef ESP8266 // NodeMCU
const int switchPin      = D5; // GPIO14
const int fingerServoPin = D6; // GPIO12
#define LED_PIN LED_BUILTIN

#else // AVR
// TODO: test
const int switchPin      = 2;
const int fingerServoPin = 5;
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

// EEPROM defines
// TODO: met struct werken en put & get & sizeof

#define EEPROM_SIZE 4
#define EEPROM_CONFIG_FLAG    0
#define EEPROM_CONFIG_TEST    1
#define EEPROM_SEQUENCE       2

#define EEPROM_CONFIG_TEST_VALUE 0x9B

#define SEQUENCE_START 0
#define SEQUENCE_END   17

static int currentseq = SEQUENCE_START;

void eeprom_reset()
{
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.println("eeprom_reset");
  DEBUG_SERIAL.flush();
#endif
  EEPROM.write(EEPROM_CONFIG_FLAG, 1);
  EEPROM.write(EEPROM_CONFIG_TEST, EEPROM_CONFIG_TEST_VALUE);
  EEPROM.write(EEPROM_SEQUENCE, SEQUENCE_START);
  EEPROM.commit();
}

void eeprom_init()
{
  EEPROM.begin(EEPROM_SIZE);
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
  EEPROM.commit();
}


void sweep(Servo *srv, int from, int to, int delayus)
{
  if (from < to)
  {
    for (int pos = from; pos < to; pos += 1)
    {
      srv->writeMicroseconds(pos);
      if (delayus > 1000)
      {
        delay(delayus / 1000);
      }
      else
      {
        delayMicroseconds(delayus);
        yield();
      }
    }
  }
  else if (from > to)
  {
    for (int pos = from; pos >= to; pos -= 1)
    {
      srv->writeMicroseconds(pos);
      if (delayus > 1000)
      {
        delay(delayus / 1000);
      }
      else
      {
        delayMicroseconds(delayus);
        yield();
      }
    }
  }
}

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

void sequence1()
{
  delay(700);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 3000);
  delay(1000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 500);
  delay(1000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid, 1800);
  sweep(&fingerServo, fingerServoMid, fingerServoTo, 500);
  delay(100);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 500);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 500);
}

void sequence2()
{
  delay(800);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid2, 3000);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(120);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 3000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 3000);
  delay(1000);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid, 1800);
  sweep(&fingerServo, fingerServoMid, fingerServoTo, 500);
  delay(100);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 500);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 500);
}

void sequence3()
{
  delay(50);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence4()
{
  delay(500);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoMid2, 1);
  delay(450);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 30000);
  delay(1);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence5()
{
  delay(1000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  delay(110);
  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  delay(110);
  sweep(&fingerServo, fingerServoTo, fingerServoMid2, 1);
  delay(110);
  sweep(&fingerServo, fingerServoMid2, fingerServoTo, 1);
  delay(110);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence6()
{
  delay(1500);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1000);
  delay(2000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1000);
  delay(2000);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence7()
{
  delay(500);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence8()
{
  delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1);
  delay(100);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 1);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence9()
{
  delay(1000);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorMid, 2000);
  delay(500);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorMid2, 1000);
  delay(1);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid3, 1);
  delay(50);
  sweep(&fingerServo, fingerServoDoorMid3, fingerServoDoorMid2, 1);
  delay(500);
  sweep(&fingerServo, fingerServoDoorMid2, fingerServoDoorMid, 5000);
  delay(1);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorTo, 1000);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 1);
  delay(450);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 1);
  delay(200);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorFrom, 1);
  delay(400);
}

void sequence10()
{
  delay(800);
  sweep(&fingerServo, fingerServoDoorFrom, fingerServoDoorTo, 30000);
  delay(1);
  sweep(&fingerServo, fingerServoFrom, fingerServoTo, 3000);
  delay(1);
  sweep(&fingerServo, fingerServoTo, fingerServoFrom, 3000);
  delay(1);
  sweep(&fingerServo, fingerServoDoorTo, fingerServoDoorMid, 30000);
  delay(1);
  sweep(&fingerServo, fingerServoDoorMid, fingerServoDoorFrom, 1);
  delay(300);
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

