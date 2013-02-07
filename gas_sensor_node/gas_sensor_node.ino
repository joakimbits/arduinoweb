/* Gas monitoring using the Arduino sensor platform.
12345678901234567890123456789012345678901234567890123456789012345678901234567890
http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/

SENSORS
MQ-7 CO sensor
- Heater 0V, 1.4V and 5V supplied from three PWM pins.
- Bias oo, 36k, 18k, 12k and 9k ohm supplied from internal pull-up resistances.
- Bias voltage drop measured by substracting pull-up voltage from 5V.
- Sensor voltage drop measured differentially in a 4-point measurement.

RESOURCES
Arduino Mega ADK http://arduino.cc/en/Hacking/PinMapping2560
- DIGITAL GND      0V >200mA sink     MQ-7 heater and sensor bias voltage
- DIGITAL 42-49    0-5V >200mA source MQ-7 heater and sensor bias voltage
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

// General settings
#define Clockrate             16000000                /* CPU rate=16 MHz. */
#define power_setup() 		  { 					  /* Configure power savings. */\
    DIDR0 = DIDR2 = 0xFF; /* Power off ADC digital input buffers. */\
	PRR0 = B10101101; /* Power off TWI Timer0:1 SPI ADC. */\
	PRR1 = B00011111; /* Power off Timer4:3 USART3:1. */
	sleep_enable(); /* Enable the sleep_cpu() instruction. */\
	set_sleep_mode(SLEEP_MODE_IDLE); /* Allow wakeup from any enabled source. */\
	sleep_bod_disable(); /* Disable VCC level detection. */\
}

// ADC control
/* http://www.atmel.com/Images/doc2549.pdf
ADMUX can be safely updated in the following ways:
1. When ADATE or ADEN is cleared.
2. During conversion, minimum one ADC clock cycle after the trigger event.
3. After a conversion, before the interrupt flag is cleared.
When updating ADMUX in one of these conditions, the new settings will affect the
next ADC conversion. Special care should be taken when changing differential 
channels. Once a differential channel has been selected, the stage may take as 
much as 125μs to stabilize to the new value. Thus conversions should not be 
started within the first 125μs after selecting a new differential channel.

The stage has a built-in offset cancellation circuitry that nulls the offset of 
differential measurements as much as possible. The remaining offset in the analog 
path can be measured directly by selecting the same channel for both differential 
inputs. This offset residue can be then subtracted in software from the 
measurement results. Using this kind of software based offset correction, offset 
on any channel can be reduced below one LSB.

The ADC features a noise canceler that enables conversion during sleep mode to 
reduce noise induced from the CPU core and other I/O peripherals. The noise 
canceler can be used with ADC Noise Reduction and Idle mode. To make use of this 
feature, the following procedure should be used:
1. Make sure that the ADC is enabled and is not busy converting. Single Conversion
mode must be selected and the ADC conversion complete interrupt must be enabled.
2. Enter ADC Noise Reduction mode (or Idle mode). The ADC will start a conversion
once the CPU has been halted.
3. If no other interrupts occur before the ADC conversion completes, the ADC 
interrupt will wake up the CPU and execute the ADC Conversion Complete interrupt 
routine. If another interrupt wakes up the CPU before the ADC conversion is 
complete, that interrupt will be executed, and an ADC Conversion Complete 
interrupt request will be generated when the ADC conversion completes. The CPU 
will remain in active mode until a new sleep command is executed.

A normal conversion takes 13 ADC clock cycles. The first conversion after the 
ADC is switched on (ADEN in ADCSRA is set) takes 25 ADC clock cycles in order to 
initialize the analog circuitry.  The actual sample-and-hold takes place 1.5 ADC 
clock cycles after the start of a normal conversion and 13.5 ADC clock cycles 
after the start of an first conversion. When a conversion is complete, the result 
is written to the ADC Data Registers, and ADIF is set. In Single Conversion mode, 
ADSC is cleared simultaneously. The software may then set ADSC again, and a new
conversion will be initiated on the first rising ADC clock edge.
Table 26-1. ADC Conversion Time
Condition                        Sample & Hold cycles   Conversion Cycles
First conversion                 13.5                   25
Normal conversions, single ended  1.5                   13
Auto Triggered conversions        2                     13.5
Normal conversions, differential  1.5/2.5               13/14
*/

#define adc_clockrate         Clockrate/128			  /* ADC clockrate 125kHz. */
#define adc_diff_settle_us    125       /* <125μs to stabilize differential net. */
#define adc_sample_us         adc_clockrate*27/2000000 /* 1.7 us first sampling time. */
#define adc_convert_us        adc_clockrate*25/1000000 /* 3.125 us first conversion time. */
#define adc_on()              ADCSRA = B10000111      /* Power on ADC. */
#define adc_start()           ADCSRA = B11001111      /* Start ADC. */
#define adc_start_sleep()     sleep_cpu()             /* Start ADC with CPU off. */
#define adc_stop()            ADCSRA = B00000111      /* Stop ADC. */
#define adc_off()             {                       /* Power off ADC. */\
	adc_stop(); PRR0 = B10101101; } 
#define adc_isrunning()       (ADCSRA & B00100000)    /* Nonzero while the ADC is running. */
#define adc_isdiff(ch)        ( 					  /* True if the channel is differential. */\
	ch >= B001000 && ch <= B011101) || (ch >= B101000 && ch <= B111101)
#define adc_set_channel(ch)   { 					  /* Set ADC channel. */\
	ADMUX = B11100000 | ch; ADCSRB = (ch & B100000) >> 2; } 
#define adc_get_channel()     (						  /* Get ADC channel. */\
	ADMUX & B1111 | (ADCSRB & B1000) << 2)


// Sensor wiring.
/* Pin mapping table
Name Functions     Arduino pin          Wiring
PK0  ADC8/PCINT16  Analog input pin 8   sensor_ref
PK1  ADC9/PCINT17  Analog input pin 9   sensor_ref
PK2  ADC10/PCINT18 Analog input pin 10  sensor_n
PK3  ADC11/PCINT19 Analog input pin 11  sensor_p
PK4  ADC12/PCINT20 Analog input pin 12  sensor_bias
PK5  ADC13/PCINT21 Analog input pin 13  sensor_bias
PK6  ADC14/PCINT22 Analog input pin 14  sensor_bias
PK7  ADC15/PCINT23 Analog input pin 15  sensor_bias
PL3  OC5A          Digital pin 46       sensor_heater
PL4  OC5B          Digital pin 45       sensor_heater
PL5  OC5C          Digital pin 44       sensor_heater
Internal pull-up resistance: 36 kOhm
The sum of all IOH for ports C0-C7, G0-G1, D0-D7, L0-L7 should not exceed 200mA.
*/

// Sensor bias control.
#define sensor_setup     PORTK /* Sensor measurement pins. */
#define sensor_output    DDRK /* Sensor measurement pin directions. */
#define sensor_status    PINK /* Sensor measurement pin values. */ 
#define sensor_ref       B00000001 /* Sensor ref = Analog input pin 8. */
#define sensor_n         B00000010 /* Sensor n = Analog input pin 9. */
#define sensor_p         B00000100 /* Sensor p = Analog input pin 10. */
#define sensor_bias      B11110000 /* Sensor bias = Analog input pins 12-15. */
#define sensor_npullups  4 /* Number of sensor bias pins. */
#define sensor_biasT     125     /* Biasing time=125us. */
#define sensor_biasR     36000   /* 36kOhm internal bias resistance */
#define sensor_biasV     5       /* 5V internal bias voltage */
#define sensor_setup() { /* Configure sensor pins. */\
  sensor_setup = 0; /* Configure all pins as either GND or floating.*/\
  sensor_output = sensor_ref; /* Connect ref pin to GND, all others floating.*/\
}
#define sensor_setBias(pullups) { /* Set sensor bias. */\
  /* byte pullups = 0..4; // Number of bias resistors to connect to 5V. */\
  sensor_setup = sensor_bias & (sensor_bias >> (sensor_npullups - pullups));\
  /* Connect selected number of 36kOhm bias resistors to 5V. */\ 
}

// Sensor measurement channels.
#define sensor_refCh     B100001 /* ADC9 */
#define sensor_nCh       B100010 /* ADC10 */
#define sensor_pCh       B100011 /* ADC11 */
#define sensor_biasCh    B100100 /* ADC12 */
#define sensor_x1Ch      B111011 /* ADC11-ADC10 1× */
#define sensor_x10Ch     B101101 /* ADC11-ADC10 10× */
#define sensor_x200Ch    B101111 /* ADC11-ADC10 200× */
#define sensor_0x1Ch     B111010 /* ADC10-ADC10 1× */
#define sensor_0x10Ch    B101100 /* ADC10-ADC10 10× */
#define sensor_0x200Ch   B101110 /* ADC10-ADC10 200× */

// Sensor heater control.
#define heater_range       ICR5     /* DAC range for digital pins 44-46. */
#define heater_value       OCR5A=OCR5B=OCR5C /* DAC value for -"-. */
#define heater_trace_vect  TIMER5_COMPA_vect /* Heater on/off event. */
#define heater_trace_on()  TIMSK5 |= B0000010 /* Enable heater tracing. */
#define heater_trace_off() TIMSK5 &= B1111101 /* Disable heater tracing. */
#define heater_setup()  { /* Enable PWM on digital pins 44-46.*/\
	TCCR5A = B11111100; TCCR5B = B00010001; }
#define heater_on()        DDRL |= B00111000 /* Enable digital pins 44-46. */
#define heater_off()       DDRL &= B11000111 /* Disable digital pins 44-46.*/
#define heater_set(value, range) { /* Set heater DAC value and range. */\
  /* word value, range; */\
  heater_off();\
  heater_range = range;\
  heater_value = value;\
  heater_on();\
}

// Schedule control.
#define schedule_setup() {                           /* Configure timer. */\
  TCCR4A = B11000000; /* Set OC4A on compare match. */\
  TCCR4B = B00000011; /* Use 64/16MHz = 4us timing resolution. */\
}
#define schedule_start() TIMSK4 = B0000010       /* Enable alarm. */
#define schedule_stop()  TIMSK4 = B0000000       /* Disable alarm. */
#define schedule_alarm   OCR4A                   /* Alarm time. */
#define schedule_vect    TIMER4_COMPA_vect       /* Alarm event. */
#define schedule_time    TCNT4                   /* Time/4us (word). */
#define schedule_ticks   65536                   /* Range 262ms/4us.*/
#define schedule_Hz      Clockrate/64            /* 1s=250000 ticks.*/ 
#define schedule_kHz     schedule_Hz/1000        /* 1ms=250 ticks.*/ 
#define schedule_MHz     schedule_kHz/1000       /* 1us=1/4 ticks.*/ 

// Sensor schedule.
#define normChannels     4 /* Normal ADC channels: ref, n, p, bias. */
#define diffChannels     2 /* Differential ADC channels: pn, nn. */
#define measure_ticks    schedule_MHz*( /* 64 measurement ticks. */\
	diffChannels*adc_diff_settle_us + /* 2*125μs */\
	(normChannels + diffChannels) * (adc_sample_us + adc_convert_us)) /* 6*5μs */
#define heat_ticks       schedule_Hz /* 250000 ticks heat control interval.  */
#define preheat_ticks    heat_ticks - measure_ticks /* 249936 ticks pre-heat. */

// Scheduler.
ISR (schedule_vect) {
	static byte skip=0; // Skip up to 255*262ms > 1 minute.
	if (skip)
		skip--;
	else {
		static register byte event = 0; // Event counter.
		switch (event++) {
			case 0: // Reset.
				schedule_stop();
				schedule_alarm = schedule_time;
			case 1: // Pre-heat at 1.4V.
				heater_set(7, 25); // = 5V * 7/25.
				skip = preheat_ticks / schedule_ticks;
				schedule_alarm += preheat_ticks;
				schedule_start();
				break;
			case 2: // Set bias. Measure ref, n, p, bias. Settle calibration. */
				sensor_set_bias(4);
				adc_on();
				adc_set_channel(sensor_refCh);
				adc_start_sleep();
				adc_set_channel(sensor_nCh);
				adc_start_sleep();
				adc_set_channel(sensor_pCh);
				adc_start_sleep();
				adc_set_channel(sensor_biasCh);
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_0x1Ch);
				schedule_alarm += (4*(adc_sample_us + adc_convert_us) + 
				                   adc_diff_settle_us)*schedule_MHz;
				break;
			case 3: // Convert calibration. Settle sensor signal. */
				adc_on();
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_x1Ch);
				schedule_alarm += (adc_sample_us + adc_convert_us + 
				                   adc_diff_settle_us)*schedule_MHz;
				break;
			case 4: // Convert sensor signal. Finish heat control . */
				schedule_alarm += (adc_sample_us + adc_convert_us)*schedule_MHz;
				adc_on();
				adc_start_sleep();
				adc_off();
				break;
			case 5: // Pre-heat at 5V.
				heater_set(25, 25); // = 5V * 25/25.
				skip = preheat_ticks / schedule_ticks;
				schedule_alarm += preheat_ticks;
				break;
			case 6: // Set bias. Measure ref, n, p, bias. Settle calibration. */
				sensor_set_bias(4);
				adc_on();
				adc_set_channel(sensor_refCh);
				adc_start_sleep();
				adc_set_channel(sensor_nCh);
				adc_start_sleep();
				adc_set_channel(sensor_pCh);
				adc_start_sleep();
				adc_set_channel(sensor_biasCh);
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_0x1Ch);
				schedule_alarm += (4*(adc_sample_us + adc_convert_us) + 
				                   adc_diff_settle_us)*schedule_MHz;
				break;
			case 7: // Convert calibration. Settle sensor signal. */
				adc_on();
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_x1Ch);
				schedule_alarm += (adc_sample_us + adc_convert_us + 
				                   adc_diff_settle_us)*schedule_MHz;
				break;
			case 8: // Convert sensor signal. Finish 5V heat control. */
				schedule_alarm += (adc_sample_us + adc_convert_us)*schedule_MHz;
				event = 1;
				adc_on();
				adc_start_sleep();
				adc_off();
				break;
		}
	}
}

// ADC handler.
volatile byte sample=255, value[256];
ISR(ADC_vect) {
	value[++sample] = ADCH;
}

void setup() {
  noInterrupts();
  power_setup();
  sensor_setup();
  heater_setup(();
  schedule_setup(();
  interrupts();
}

/* PROTOCOL */

/* APPLICATION */

#define SampleInterval 10000 /*ms*/

void loop() {
  static unsigned long t0 = millis();
  if( millis() - t0 > SampleInterval ) {
    OS_2_1_THpacket( 2, edge, magnitudes[edge] );
    t0 += SampleInterval;
  }
}
