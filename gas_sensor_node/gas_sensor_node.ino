/* Gas monitoring using the Arduino sensor platform in an arduously efficient manner.
12345678901234567890123456789012345678901234567890123456789012345678901234567890
http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/

SENSORS
MQ-7 CO sensor
- Heater 0V, 1.4V and 5V supplied from three PWM pins.
- Bias oo, 36k, 18k, 12k and 9k ohm from 5V supplied from internal pull-up resistances.
- Bias voltage drop measured by substracting pull-up voltage from 5V.
- Sensor voltage drop measured differentially in a 4-point measurement.
- timeH, timeL and value for ref, n, p, bias, nnx1 and pnx1 channels sent at 76800 baud.

RESOURCES
- DIGITAL GND      0V >200mA sink       MQ-7 heater reference.
- DIGITAL 44-46    0-5V >200mA source   MQ-7 heater voltage.
- ANALOG IN 12-15  36kOhm pullups to 5V MQ-7 bias voltage.
- ANALOG IN 11     ADC11                MQ-7 sensor voltage, bias side.
- ANALOG IN 10     ADC10                MQ-7 sensor voltage, reference side.
- ANALOG IN 8-9    0V >100mA sink       MQ-7 sensor reference.
- PWM 13           LED source           MQ-7 activity indicator (not yet used).
- ADC              0-5V input           MQ-7 bias and sensor voltage measurements.
- TIMER4           250kHz timer			MQ-7 scheduler.
- TIMER5           16MHz timer			MQ-7 heater PWM.
- RAM              <1kB                 MQ-7 static variables.
- USB              5V, COM              Power supply and host control.
*/

// Power control
#define power_setup() 		  { 					  /* Configure power savings. */\
    DIDR0 = DIDR2 = 0xFF; /* Power off ADC digital input buffers. */\
	PRR0 = B10101101; /* Power off TWI Timer0:1 SPI ADC. */\
	PRR1 = B00011111; /* Power off Timer4:3 USART3:1. */
	sleep_enable(); /* Enable the sleep_cpu() instruction. */\
	set_sleep_mode(SLEEP_MODE_IDLE); /* Allow wakeup from any enabled source. */\
	sleep_bod_disable(); /* Disable VCC level detection. */\
}

// UART control
#define uart_baudrate        76800 /* Highest good baudrate on Arduino is 76800. */
#define _uart_ubrr           12 /* F_CPU/16/uart_baudrate - 1 */
#define _uart_baudrate       F_CPU/16/(_uart_ubrr + 1) /* 76923.08 */
#define uart_convert_us      130 /* 10000000/_uart_baudrate */ 
#define uart_setup()         Serial.begin(uart_baudrate)
#define uart_set(c)          UDR = c /* Transmit character. */
#define uart_get()           UDR /* Get received character. */

// ADC control
#define adc_clockrate         F_CPU/128			  /* ADC clock = 125kHz. */
#define adc_diff_settle_us    125       /* <125μs to stabilize differential net. */
#define adc_first_sample_us   adc_clockrate*27/2000000 /* 1.7 us first sampling time. */
#define adc_first_convert_us  adc_clockrate*25/1000000 /* 3.125 us first conversion time. */
#define adc_sample_us         adc_clockrate*3/2000000  /* 0.1875 us normal sampling time. */
#define adc_convert_us        adc_clockrate*13/1000000 /* 1.625 us normal conversion time. */
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

// Sensor bias control
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

// Sensor measurement channels
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

// Sensor heater control
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

// Schedule control
#define schedule_setup() {                           /* Configure timer. */\
  TCCR4A = B11000000; /* Set OC4A on compare match. */\
  TCCR4B = B00000011; /* Use 64/16MHz = 4us timing resolution. */\
}
#define schedule_start() TIMSK4 = B0000010       /* Enable alarm. */
#define schedule_stop()  TIMSK4 = B0000000       /* Disable alarm. */
#define schedule_set(t)  OCR4A = t               /* Set alarm time. */
#define schedule_vect    TIMER4_COMPA_vect       /* Alarm event. */
#define schedule_time()  TCNT4                   /* Time/4us (word). */
#define schedule_ticks   65536                   /* Range 262ms/4us.*/
#define schedule_Hz      F_CPU/64                /* 1s=250000 ticks.*/ 
#define schedule_kHz     schedule_Hz/1000        /* 1ms=250 ticks.*/ 
#define schedule_MHz     schedule_kHz/1000       /* 1us=1/4 ticks.*/ 

// Sensor schedule
#define normChannels     4 /* Normal ADC channels: ref, n, p, bias. */
#define diffChannels     2 /* Differential ADC channels: pn, nn. */
#define channels         (normChannels + diffChannels)
#define uart_ticks       33 /* uart_convert_us*schedule_MHz < 33 ticks */
#define tx_ticks         channels * 3 * uart_ticks /* UART Tx time = 594 ticks (148.5μs). */
#define first_ticks      (adc_first_sample_us + adc_first_convert_us)*schedule_MHz
#define norm_ticks       (adc_sample_us + adc_convert_us)*schedule_MHz
#define diff_ticks       (adc_diff_settle_us + adc_first_sample_us + adc_first_convert_us)*schedule_MHz
#define measure_ticks    (first_ticks + (normChannels-1)*norm_ticks + diff_ticks)
#define min_task_ticks   (tx_ticks + measure_ticks) /* Min task time = 662 ticks (165.5μs). */

// Scheduler
volatile byte samples=0
volatile int time[channels];
volatile byte value[channels];
ISR(ADC_vect) {
	value[samples++] = ADCH;
}
ISR (schedule_vect) {
	/*
	Alternate through two heat settings: heat 1s*5V and 1s*1.4V. 
	Send timeH, timeL and value of each ADC conversion to the uart at 76800 baud.
	Alternate through five bias settings: oo, 36kOhm, 18kOhm, 12kOhm and 9kOhm.
	Measure six channels: ref, n, p, bias, nnx1 and pnx1.
	*/
	static word skip=0; // Skip up to 0x1000*262ms > 4 hours.
	if (skip)
		skip--;
	else {
		static register char task = -1; // Task counter.
		static register char event = -1; // Event counter.
		static register char sample = -1; // Sample counter.
		static register char bias = -1; // Bias counter.
		static word alarm; // Alarm tick.
		static word endtime; // End-of-task tick.
		long ticks;
		switch (++event) {
			case 0: // Reset.
				alarm = schedule_time();
			case 1: // Alternate through heat using 5V and 1.4V.
				switch(task = ++task % 2) {
					case 0: // 1s * 5V
						heater_set(25, 25);
						ticks = schedule_Hz;
						break;
					case 1: // 1s * 1.4V
						heater_set(7, 25);
						ticks = schedule_Hz;
						break;
				}
				endtime = alarm + ticks;
				sample = 0;
			case 2: // Send sampled timeH.
				if (samples) {
					uart_set(time[sample] >> 8);
					schedule_set(alarm += uart_ticks);
					break;
				}
			case 3: // Send sampled timeL.
				if (samples) {
					uart_set(time[sample]);
					schedule_set(alarm += uart_ticks);
					break;
				}
			case 4: // Send sampled value.
				if (samples) {
					uart_set(value[sample]);
					schedule_set(alarm += uart_ticks);
					samples--;
					sample++;
					ticks -= 3 * uart_ticks;
					event = 1; // Next event = 2.
					break;
				}
			case 5: // Wait for pre-heating to complete.
				schedule_set(alarm += ticks);
				skip = ticks / schedule_ticks;
				break;
			case 6: // Alternate through bias 0-4. Measure ref, n, p, bias. Settle calibration. */
				sensor_set_bias(bias = ++bias % 5);
				adc_on();
				adc_set_channel(sensor_refCh);
				time[0] = schedule_time() - endtime;
				adc_start_sleep();
				adc_set_channel(sensor_nCh);
				time[1] = schedule_time() - endtime;
				adc_start_sleep();
				adc_set_channel(sensor_pCh);
				time[2] = schedule_time() - endtime;
				adc_start_sleep();
				adc_set_channel(sensor_biasCh);
				time[3] = schedule_time() - endtime;
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_0x1Ch);
				schedule_set(alarm = endtime - 2 * diff_ticks);
				break;
			case 7: // Convert calibration. Settle sensor signal. */
				adc_on();
				time[4] = schedule_time() - endtime;
				adc_start_sleep();
				adc_off();
				adc_set_channel(sensor_x1Ch);
				schedule_set(alarm += endtime - diff_ticks);
				break;
			case 8: // Convert sensor signal. Finish heat control . */
				schedule_set(alarm = endtime);
				event = 0; // Next event = 1.
				adc_on();
				time[5] = schedule_time() - endtime;
				adc_start_sleep();
				adc_off();
				break;
		}
	}
}

void setup() {
  noInterrupts();
  power_setup();
  uart_setup();
  sensor_setup();
  heater_setup(();
  schedule_setup(();
  interrupts();
}

void loop() {
}
