/* Gas monitoring using the Arduino sensor platform.
12345678901234567890123456789012345678901234567890123456789012345678901234567890
http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/

SENSORS
MQ-7 CO sensor
- Symmetrically connected using 10*470=4k7 ohm total bias resistance.

RESOURCES
Arduino Mega ADK
- DIGITAL GND      0V >200mA sink     MQ-7 heater and sensor bias voltage
- DIGITAL 22-49    0-5V >200mA source MQ-7 heater and sensor bias voltage
- ANALOG IN 14-15  Differential input MQ-7 sensor voltage
- PWM 13           LED source         Activity indicator
- USB              5V, COM            Power supply and host control
- ADC              0-5V input         Measure analog sensors
- TIMER2           256 bits 16MHz     Controller timing
- STATIC RAM       >2 kB              1 kB cyclic buffers

SERVICES
Controller
- Reads bias voltage values and wait intervals from 256-cyclic buffers 
'bias[tick]' and 'wait[tick]' respectively, until wait[tick]=0. 
- Stores adc values measured immediately before each change into 'values[tick]'.
Server
- Transfers interleaved bias voltages and wait intervals from serial port to 
controller.
- Transfers adc values from controller to serial port.
*/

#define BasebandURL           'http://localhost' /* Remote baseband. */
#define SignalAnPin           0   /* Signal-strength analog pin. */
#define SignalThreshold       19   /* Threshold for streaming. */
#define SignalDetectedPin     3   /* Signal-detected pin. */
#define SignalOutputPin       2   /* Signal-output pin. */
#define StreamPin             13  /* Stream-active pin. */

#define adcON()               ADCSRA = B11111100      /* Switch on ADC. */
#define adcOFF()              ADCSRA = B00000000      /* Switch off ADC. */
#define adcSelect(ch)         ADMUX = B01100000 | ch  /* Select ADC channel. */
#define Clockrate             16000000/256            /* 16MHz/256 = 62500Hz. */
#define clock()               TCNT2                   /* Clock register. */
#define rxAlarm_vect          TIMER2_COMPA_vect       /* Receiver watchdog. */
#define txAlarm_vect          TIMER2_COMPB_vect       /* Transmitter timing. */
#define rxAlarm               OCR2A                   /* Rx alarm register. */
#define txAlarm               OCR2B                   /* Tx alarm register. */
#define rxAlarmON()           TIMSK2 |= B0000010      /* Enable rx alarms. */
#define rxAlarmOFF()          TIMSK2 &= B1111101      /* Disable rx alarms. */
#define txAlarmON()           TIMSK2 |= B0000100      /* Enable tx alarms. */
#define txAlarmOFF()          TIMSK2 &= B1111011      /* Disable tx alarms. */
#define Baudrate              1200                    /* RF-symbol rate. */
#define Clockticks            12                      /* Ticks per clock step. */
#define OS_2_1_interval       4*Clockticks*Clockrate/Baudrate
                                                      /* Ticks per bit = 2500. */

volatile byte cycles[256];           // Clock cycles between each received edges.
volatile char magnitudes[256];      // ADC value changes of the signal.
volatile byte edge = 255;            // Last received edge.

volatile byte ticks[256];            // Ticks until next edge to transmit.
volatile byte edge0 = 0;             // Last edge transmitted.
byte edge1 = 0;                      // Last edge to transmit.
byte edge2 = 0;                      // Last edge defined.

void setup() {
  // Setup I/O pins.
  pinMode( SignalDetectedPin, INPUT );
  pinMode( SignalOutputPin, OUTPUT );
  pinMode( StreamPin, OUTPUT );
  noInterrupts();
  attachInterrupt( SignalDetectedPin - 2, receiver, CHANGE );
  // ADC settings - see ATmega48/88/168 pp. 244-259.
  adcSelect(SignalAnPin);
  adcON();
  ADCSRB = 0;                                // Use auto-trigger.
  DIDR0  = B00111111;                        // Disable unused digital buffers.
  // Timer settings -- see ATmega48/88/168 pp. 131-132. 
  TCCR2A = B1111000;                         // Enable alarm comparators.
  TCCR2B = B0000110;                         // Clockrate = 16MHz/256.
  TIMSK2 = B0000000;                         // Disable alarms.
  interrupts();
}

/* RECEIVER */

byte t1;

void receiver() {
  // On signal-detected pin change event:
  //   Fetch digital signal, start analog signal measurement and start watchdog.
  static byte t0 = clock();                  // Previous cycle.
  byte t = clock();                          // Current cycle.
  if (digitalRead( SignalDetectedPin ) ^ (edge & 1))
      cycles[++edge] = 0;                    // Snap to odd/even edge.
  cycles[++edge] = t - t0;                   // Store cycles.
  t0 = t;                                    // Remember cycle.
  adcOFF();                                  // Abort ADC.
  magnitudes[edge] = 0;                      // Clear analog reading.
  t1 = t + 1;
  adcON();                                   // Start ADC.
  rxAlarm = t + OS_2_1_interval/Clockrate;   // Move watchdog.
  rxAlarmON();                               // Enable watchdog.
}

ISR(ADC_vect) {
  // On ADC-complete event:
  //   Fetch analog signal measurement and disable ADC.
  byte ADCch = ADMUX & B00000111;
  if( clock() > t1 && ADCch == SignalAnPin ) {
    static char v0 = 0;
    char v = ADCH;
    char dv = magnitudes[edge] = v - v0;
    if (v0 && abs(dv) > SignalThreshold) {
      digitalWrite( StreamPin, digitalRead(StreamPin) ^ 1 );
    }
    v0 = v;
    adcOFF();
  }
}

ISR(rxAlarm_vect) {
  // On rxAlarm watchdog event:
  //   Mark end-of-packet, disable ADC and disable watchdog.
  cycles[++edge] = 0;                      // Mark end-of-packet.
  adcOFF();                                // Abort ADC.
  rxAlarmOFF();                            // Disable watchdog.
}

/* TRANSMITTER */

ISR(txAlarm_vect) {
  // On txAlarm event:
  //   Transmit next edge0 and move alarm until edge1 is reached.
  if( edge0 != edge1 ) {                        // If data remaining:
    edge0++;                                      // Move to next edge.
    digitalWrite( SignalOutputPin, edge0 & 1 );   // Transmit data.
    static byte t = Clockticks/2;                 // Start with one half-cycle.
    t += ticks[edge0];                            // Add target ticks.
    byte c = t/Clockticks;                        // Find nearest clock cycle.
    txAlarm += c;                                 // Move alarm to that cycle.
    t -= c * Clockticks;                          // Substract used ticks.    
   } else txAlarmOFF();                         // Otherwise disable alarm.
}

/* PROTOCOL */

byte OS_2_1_bits( byte *buf, byte bits, boolean transmit ) {
  // Add edge2 ticks to transmit for each bit in 'buf', LSB first. Transferred to
  // the transmitter when 'tranmit' is set. Returns the 8-bit sum of the nibbles
  // in 'buf'. 
  //   The edges are added according to the Oregon Scientific 2.1 sensor data
  // packet format, where a bit change is represented by an 1010 signal and
  // a bit without change is represented by 1100.   
  static byte b0 = 0;                           // Previous bit.
  byte bytes = (bits + 7) / 8;                  // Calculate number of bytes.
  byte checksum = 0;                            // Prepare checksum.
  for( ; bytes; bytes--, buf++ ) {              // For each byte in the buffer:
    checksum += *buf >> 4 + *buf & 0xF;           // Add nibbles to the checksum.
    for( byte n = 0; bits-- && n <= 7; n++ ) {    // For each bit in the byte:
      byte b = *buf & 1 << n? 1: 0;                 // Extract the bit.
      byte di = 2 - (b ^ b0);                       // # symbols between edges.
      for( byte i = 0; i < 4; i += di )
          ticks[edge2++] = di*OS_2_1_interval/4;    // Add edges.
      b0 = b;                                       // Remember the bit.
    }
  }
  if( transmit ) {                              // If ready to transmit:
    byte e1 = edge1;                              // Read end-of-transmission.
    edge1 = edge2;                                // Move end-of-transmission.
    if( edge0 == e1 ) {                           // If transmitter is idle:
      edge0--;                                      // Start from edge before.
      txAlarm = clock()+OS_2_1_interval/Clockticks; // Position first edge.
      txAlarmON();                                  // Start the transmitter.
    }
    b0 = 0;                                       // Reset bit-comparisons.
  }
  return checksum;
}

void OS_2_1_packet( word sensorID, 
                    byte channel, 
                    byte rollingCode, 
                    byte flags, 
                    byte* data,
                    byte databits) {
  // Add edge1 ticks representing an Oregon Scientific 2.1 sensor 
  // data packet.
  word sync = 0x5555;
  byte checksum = 0;
  byte zero = 0;
  OS_2_1_bits( (byte*) &sync, 16, false );
  checksum += OS_2_1_bits( (byte*) &sensorID, 16, false );
  checksum += OS_2_1_bits( &channel, 4, false );
  checksum += OS_2_1_bits( &rollingCode, 8, false );
  checksum += OS_2_1_bits( &flags, 4, false );
  checksum += OS_2_1_bits( data, databits, false );
  OS_2_1_bits( &checksum, 8, false );
  OS_2_1_bits( &zero, 8, true );
}

#define THsensorID 0x02D1

char OS_2_1_THpacket( byte channel, int T, byte H ) {
  // Add edge1 ticks representing an Oregon Scientific 2.1 temperature
  // and humidity sensor data packet. 
  // Temperature unit: 0.1 degC. Relative humidity unit: %.
  byte data[3];
  boolean Ts = T < 0;
  T = abs(T);
  data[0] = (byte)(T % 10) | (byte)(T / 10 % 10) << 4;
  data[1] = (byte)(T / 100 % 10) | (byte)Ts << 4;
  data[2] = H % 10 | (H / 10 % 10) << 4;
  OS_2_1_packet( THsensorID, channel, 0, 0, data, 6*4 );
}

/* APPLICATION */

#define SampleInterval 10000 /*ms*/

void loop() {
  static unsigned long t0 = millis();
  if( millis() - t0 > SampleInterval ) {
    OS_2_1_THpacket( 2, edge, magnitudes[edge] );
    t0 += SampleInterval;
  }
}
