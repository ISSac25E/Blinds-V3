#include "Arduino.h"

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A0, LOW);
  digitalWrite(A2, HIGH);

  timer2Setup(0);
}

uint32_t timer = millis();
uint32_t potVal = 0;

void loop()
{
  if (millis() - timer >= 30) {
    potVal = map(analogRead(A1), 0, 1023, 500, 5000);
    timer = millis();
    Serial.println(potVal);
  }
}

volatile uint8_t secondaryPrescaler = 0;
volatile uint8_t prescaleCount = 0;

// lookup tables for timer intervals starting with prescaler 8
volatile const uint8_t __timerTable_A__[] PROGMEM =
    {
        0,
        1,
        2,
        3,
        4,
        6,

        8,
        10,
        12};
volatile const uint32_t __timerTable_B__[] PROGMEM =
    {
        128,
        512,
        1024,
        2048,
        4096,
        16384,

        65536,
        262144,
        1048576};

void timer2Setup(uint32_t interval_us)
{
  volatile uint8_t timerPrescaleSelect;

  for (timerPrescaleSelect = 0; timerPrescaleSelect < (sizeof(__timerTable_B__) >> 2); timerPrescaleSelect++)
  {
    if (interval_us <= pgm_read_dword_near(__timerTable_B__ + timerPrescaleSelect))
      break;
  }

  volatile uint8_t prescalerSet = (timerPrescaleSelect < 6 ? timerPrescaleSelect + 2 : B111);

  cli();
  secondaryPrescaler = (timerPrescaleSelect < 6 ? 0 : 1 << (pgm_read_byte_near(__timerTable_A__ + timerPrescaleSelect) - 6));
  prescaleCount = 0;

  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 = 0;

  OCR2A = (timerPrescaleSelect ? (interval_us >> pgm_read_byte_near(__timerTable_A__ + timerPrescaleSelect)) - 1 : (interval_us << 1) - 1);

  TCCR2A |= (1 << WGM21);

  TCCR2B |= ((prescalerSet & B100 ? 1 : 0) << CS22) | ((prescalerSet & B010 ? 1 : 0) << CS21) | ((prescalerSet & B001 ? 1 : 0) << CS20);

  TIMSK2 |= (1 << OCIE2A);
  TCNT2 = 0;
  sei();
}

ISR(TIMER2_COMPA_vect)
{
  if (prescaleCount < secondaryPrescaler)
    prescaleCount++;
  else
  {
    prescaleCount = 0;
    digitalWrite(13, !digitalRead(13));
    timer2Setup(potVal);
  }
}
