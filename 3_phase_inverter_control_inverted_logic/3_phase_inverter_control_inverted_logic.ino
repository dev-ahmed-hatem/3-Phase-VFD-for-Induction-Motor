#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Sine table (64 entries)
const uint8_t sinTable_64[64] PROGMEM = {
  128, 140, 153, 165, 177, 188, 199, 209, 218, 226, 234, 240, 245, 249, 252, 254,
  255, 254, 252, 249, 245, 240, 234, 226, 218, 209, 199, 188, 177, 165, 153, 140,
  128, 115, 102, 90, 78, 67, 56, 46, 37, 29, 21, 15, 10, 6, 3, 1,
  0, 1, 3, 6, 10, 15, 21, 29, 37, 46, 56, 67, 78, 90, 102, 115
};

// const uint8_t sinTable_60[60] PROGMEM = {
//     128, 141, 154, 167, 179, 191, 202, 212, 222, 231, 239, 246, 252, 255, 255,
//     255, 255, 255, 252, 246, 239, 231, 222, 212, 202, 191, 179, 167, 154, 141,
//     128, 115, 102, 89, 77, 65, 54, 44, 34, 25, 17, 10, 4, 0, 0,
//     0, 0, 0, 4, 10, 17, 25, 34, 44, 54, 65, 77, 89, 102, 115
// };

volatile uint8_t phaseA = 0;
volatile uint8_t phaseB = 21;
volatile uint8_t phaseC = 42;

// Dead-time counters
volatile uint8_t dtA = 0, dtB = 0, dtC = 0;

// Previous states
volatile uint8_t prevA = 0, prevB = 0, prevC = 0;

#define DEAD_TIME_CYCLES 1

// Speed control
volatile uint8_t speedDivider = 4;

void setup() {
  // PWM pins
  DDRB |= (1 << PB1) | (1 << PB2);  // Phase A (D9 -> HIGH    |  D10 -> LOW)
  DDRD |= (1 << PD6) | (1 << PD5);  // Phase B (D6 -> HIGH    |  D5  -> LOW)
  DDRB |= (1 << PB3);               // Phase C high (D11 -> HIGH)
  DDRD |= (1 << PD3);               // Phase C low  (D3  -> LOW)

  cli();

  // Timer1 → Phase A
  // waveform generation mode: PWM, phase correct, 8-bit
  // compare output mode: inverting
  TCCR1A = (1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM10);
  // no prescalar
  TCCR1B = (1 << CS10);

  // Timer0 → Phase B
  // waveform generation mode: PWM, phase correct
  // compare output mode: inverting
  TCCR0A = (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0) | (1 << WGM00);
  // no prescalar
  TCCR0B = (1 << CS00);


  // Timer2 → Phase C + ISR trigger
  // waveform generation mode: PWM, phase correct
  // compare output mode: inverting
  TCCR2A = (1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0) | (1 << WGM20);
  // no prescalar
  TCCR2B = (1 << CS20);


  TIMSK1 |= (1 << TOIE1);  // overflow interrupt

  sei();
}

ISR(TIMER1_OVF_vect) {
  static uint8_t updateCounter = 0;

  if (++updateCounter < speedDivider) return;
  updateCounter = 0;

  uint8_t a = pgm_read_byte(&sinTable_64[phaseA]);
  uint8_t b = pgm_read_byte(&sinTable_64[phaseB]);
  uint8_t c = pgm_read_byte(&sinTable_64[phaseC]);

  uint8_t stateA = (a > 128);
  uint8_t stateB = (b > 128);
  uint8_t stateC = (c > 128);

  // ===== PHASE A =====
  if (stateA != prevA) dtA = DEAD_TIME_CYCLES;

  if (dtA > 0) {
    OCR1A = 0;
    OCR1B = 0;
    dtA--;
  } else {
    OCR1A = stateA ? a : 0;
    OCR1B = stateA ? 0 : (255 - a);
  }
  prevA = stateA;

  // ===== PHASE B =====
  if (stateB != prevB) dtB = DEAD_TIME_CYCLES;

  if (dtB > 0) {
    OCR0A = 0;
    OCR0B = 0;
    dtB--;
  } else {
    OCR0A = stateB ? b : 0;
    OCR0B = stateB ? 0 : (255 - b);
  }
  prevB = stateB;

  // ===== PHASE C =====
  if (stateC != prevC) dtC = DEAD_TIME_CYCLES;

  if (dtC > 0) {
    OCR2A = 0;
    OCR2B = 0;
    dtC--;
  } else {
    OCR2A = stateC ? c : 0;
    OCR2B = stateC ? 0 : (255 - c);
  }
  prevC = stateC;

  // Increment phases
  phaseA = (phaseA + 1) & 63;
  phaseB = (phaseB + 1) & 63;
  phaseC = (phaseC + 1) & 63;
}

void loop() {
  uint16_t val = analogRead(A0);

  // Smaller = faster
  // 50 ~ 100 Hz
  speedDivider = map(val, 0, 1023, 10, 40);
  // 2 ~ 10 Hz
  // speedDivider = map(val, 0, 1023, 100, 500);
}