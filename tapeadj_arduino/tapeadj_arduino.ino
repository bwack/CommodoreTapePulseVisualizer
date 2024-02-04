// Measures the HIGH width, LOW width, frequency, and duty-cycle of a pulse train
// on Arduino UNO Pin 8 (ICP1 pin).  

// Note: Since this uses Timer1, Pin 9 and Pin 10 can't be used for
// analogWrite().

/*
 * BAR1: D2, PD2
 * BAR2: D3, PD3
 * BAR3: D4, PD4
 * BAR4: D5, PD5
 * BAR5: D6, PD6
 * BAR6: D7, PD7
 * 
 * LED0: A0, PC0
 * LED1: A1, PC1
 * LED2: A2, PC2
 * LED3: A3, PC3
 * LED4: A4, PC4
 * LED5: A5, PC5
 * LED6: D9, PB1
 * LED7: D10, PB2
 * LED8: D11, PB3
 * LED9: D12, PB4
 * 
 * CASSRD: D8 (ICP1/PB0). Special interrupt capture pin
 * 
 * Note: A6 and A7 are inputs only.
 */


#define BAR1 PD2
#define BAR2 PD3
#define BAR3 PD4
#define BAR4 PD5
#define BAR5 PD6
#define BAR6 PD7
#define LED0 PC0

char a = 1;
char plex = 1;

void display(uint8_t v)
{
  uint8_t val=v%10;
  // Serial.println(v);
  // Serial.println(val);
  if (val<6) {
    PORTC=1<<val;
    PORTB=0;
  } else {
    PORTB= 1<<(val-5);
    PORTC=0;
  }

  PORTD=0;
  if (v<10) {
    DDRD = 1<<BAR1;
  }
  else if (v<20) {
    DDRD = 1<<BAR2;
  }
  else if (v<30) {
    DDRD = 1<<BAR3;
  }
  else if (v<40) {
    DDRD = 1<<BAR4;
  }
  else if (v<50) {
    DDRD = 1<<BAR5;
  }
  else {
    DDRD = 1<<BAR6;    
  }

}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  char count=0;
  DDRC = 0b00111111;
  DDRD = 0b11001100;
  DDRB = 0b00011110;
  for (uint8_t count=0; count<60; count++) {
    display(count);
    delay(40);
  }
  delay(100);

  // For testing, uncomment one of these lines and connect
  // Pin 3 or Pin 5 to Pin 8
  // analogWrite(3, 64);  // 512.00, 1528.00, 2040.00, 25.10%, 490.20 Hz
  // analogWrite(5, 64);  // 260.00, 764.00, 1024.00, 25.39%, 976.56 Hz

  noInterrupts ();  // protected code
  // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TIMSK1 = 0;

  TIFR1 |= _BV(ICF1); // clear Input Capture Flag so we don't get a bogus interrupt
  TIFR1 |= _BV(TOV1); // clear Overflow Flag so we don't get a bogus interrupt

  TCCR1B = _BV(CS10) | // start Timer 1, no prescaler
           _BV(ICES1); // Input Capture Edge Select (1=Rising, 0=Falling)

  TIMSK1 |= _BV(ICIE1); // Enable Timer 1 Input Capture Interrupt
  TIMSK1 |= _BV(TOIE1); // Enable Timer 1 Overflow Interrupt
  interrupts ();
}

volatile uint32_t PulseHighTime = 0;
volatile uint32_t PulseLowTime = 0;
volatile uint16_t Overflows = 0;

ISR(TIMER1_OVF_vect)
{
  Overflows++;
}

ISR(TIMER1_CAPT_vect)
{
  static uint32_t firstRisingEdgeTime = 0;
  static uint32_t fallingEdgeTime = 0;
  static uint32_t secondRisingEdgeTime = 0;

  uint16_t overflows = Overflows;

  // If an overflow happened but has not been handled yet
  // and the timer count was close to zero, count the
  // overflow as part of this time.
  //if ((TIFR1 & _BV(TOV1)) && (ICR1 < 1024))
  if ((TIFR1 & _BV(TOV1)))
    overflows++;

  if (PulseLowTime == 0)
  {
    if (TCCR1B & _BV(ICES1))
    {
      // Interrupted on Rising Edge
      if (firstRisingEdgeTime)
      {
        // ... so this is the second rising edge, ending the low part
        // of the cycle.
        secondRisingEdgeTime = overflows; // Upper 16 bits
        secondRisingEdgeTime = (secondRisingEdgeTime << 16) | ICR1;
        PulseLowTime = secondRisingEdgeTime - fallingEdgeTime;
        firstRisingEdgeTime = 0;
      }
      else
      {
        firstRisingEdgeTime = overflows; // Upper 16 bits
        firstRisingEdgeTime = (firstRisingEdgeTime << 16) | ICR1;
        TCCR1B &= ~_BV(ICES1); // Switch to Falling Edge
      }
    }
    else
    {
      // Interrupted on Falling Edge
      fallingEdgeTime = overflows; // Upper 16 bits
      fallingEdgeTime = (fallingEdgeTime << 16) | ICR1;
      TCCR1B |= _BV(ICES1); // Switch to Rising Edge
      PulseHighTime = fallingEdgeTime - firstRisingEdgeTime;
    }
  }
}

void loop()
{

  noInterrupts();
  uint32_t pulseHighTime = PulseHighTime;
  uint32_t pulseLowTime = PulseLowTime;
  interrupts();

  // If a sample has been measured
  if (pulseLowTime)
  {
/*
    if ((pulseHighTime>>4 < 200) && (pulseHighTime>>4 > 120 )) {
    // Display the pulse length in microseconds
    Serial.print("High time (microseconds): ");
    Serial.println(pulseHighTime / 16.0, 2);
    Serial.print("Low time (microseconds): ");
    Serial.println(pulseLowTime / 16.0, 2);

    uint32_t cycleTime = pulseHighTime + pulseLowTime;
    Serial.print("Cycle time (microseconds): ");
    Serial.println(cycleTime / 16.0, 2);
    Serial.println(cycleTime>>4);

    float dutyCycle = pulseHighTime / (float)cycleTime;
    Serial.print("Duty cycle (%): ");
    Serial.println(dutyCycle * 100.0, 2);

    float frequency = (float)F_CPU / cycleTime;
    Serial.print("Frequency (Hz): ");
    Serial.println(frequency, 2);
    Serial.println();

    //uint32_t diff = 8 + 1000 - cycleTime/16;
    int32_t pulselen_measured = pulseHighTime>>4;
    uint8_t factual_pulselen = pulselen_measured > 155 ? 183 : 130;
    int32_t diff = ( pulselen_measured*100 - factual_pulselen*100)/factual_pulselen;
    diff = 8 - diff/2;
    Serial.println(diff);
    Serial.println(pulselen_measured);

*/    
    display(PulseHighTime>>7);
    delay(1);

    // Request another sample
    noInterrupts();
    PulseLowTime = 0;
    interrupts();
  }

}
