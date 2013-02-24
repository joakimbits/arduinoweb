/* Gas monitoring using ATMega2560, developed on Arduino Mega ADK.
12345678901234567890123456789012345678901234567890123456789012345678901234567890

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

/* DRIVERS *********************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// Power control
#define power_setup() {           /* Configure power savings. */\
    DIDR0 = DIDR2 = 0xFF;             /* Power off ADC digital input buffers. */\
    PRR0 = B10101101;                 /* Power off TWI Timer0:1 SPI ADC. */\
    PRR1 = B00001111;                 /* Power off Timer3 USART3:1. */\
    set_sleep_mode(SLEEP_MODE_IDLE);  /* Allow wakeup from any enabled source. */\
    sleep_enable();                   /* Enable the sleep_cpu() instruction. */\
}

// Status control
#include <Ethernet.h>
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x09, 0xE3 };
EthernetServer status(80);
#define server_SD_WProtectAnPin   0  /* A0 */
#define server_SD_DetectAnPin     1  /* A1 */
#define server_IntPin             2  /* D2 */
#define server_SD_SPI_CSPin       4  /* D4 */
#define server_SPI_CSPin          10 /* D10 */
#define server_SPI_MOSIPin        11 /* D11 */
#define server_SPI_MISOPin        12 /* D12 */
#define server_SPI_SCKPin         13 /* D13 */
#define server_setup() {          /* Start status server. */
    Ethernet.begin(mac);\
    status.begin();\
}
#define server_address()          Ethernet.localIP()

// Datastream control
#define data_baudrate         76800 /* Highest baudrate with <1% timing error. */
#define _data_ubrr            12 /* F_CPU/16/data_baudrate - 1) = 12.02. */
#define data_us               10*(_data_ubrr+1)*16*1000000/F_CPU. /* 130us/byte. */ 
#define data_setup() {        /* Configure serial communication using USART0. */\
    UBRR0L = _data_ubrr;          /* F_CPU/16/(_data_ubrr+1) = 76923.1 baud. */\
    UCSR0B = B00011000;           /* Use RxD0 & TxD0 pins. */\
    UCSR0C = B00000011;           /* 8 bits with no parity and 1 stop bit. */\
}
#define data_receiver_vect    USART0_RX_vect /* Receive event. */
#define data_receiver_start() UCSR0B |= B10000000 /* Enable receive interrupts. */
#define data_receiver_stop()  UCSR0B &= B01111111 /* Disable recieve interrupts. */
#define data_receive()        UDR0 /* Get received byte. */
#define data_transmitter_vect USART0_UDRE_vect /* Transmit event. */
#define data_transmitter_start() UCSR0B |= B00100000 /* Enable transmitter interrupts. */
#define data_transmitter_stop()  UCSR0B &= B11011111 /* Disable transmitter interrupts. */
#define data_transmit(c)      UDR0 = c /* Transmit byte. */
volatile byte data_read_buf[]; // Receive buffer.
volatile byte data_read_i = 0; // Receive buffer index.
volatile byte data_read_o = 0; // Receive buffer size.
#define data_receiving()      data_read_o? data_read_i < data_read_o: data_read_buf[data_read_i]
ISR(data_receiver_vect) {     // On receive ready events: Receive buffer and stop.
    data_read_buf[data_read_i++] = data_receive();
	if (!data_receiving())
	    data_receiver_stop();
}
#define data_read_wait()      while (data_receiving()) sleep_cpu()
#define data_read(buf,n) {  /* Set receive buffer (n bytes or string if n=0) and start receiver. */\
	data_read_wait();\
	data_read_buf = (byte*)buf;\
	data_read_o = n;\
	data_read_i = 0;\
	data_receiver_start();\
}
volatile byte data_write_buf[]; // Transmit buffer.
volatile byte data_write_i = 0; // Transmit buffer size.
volatile byte data_write_o = 0; // Transmit buffer index.
#define data_transmitting()   data_write_i? data_write_o < data_write_i: data_write_buf[data_write_o]
ISR(data_transmitter_vect) {  // On transmit ready events: Transmit buffer and stop.
    if (data_transmitting())
	    data_transmit(data_write_buf[data_write_o++]);
	else
	    data_transmitter_stop();
}
#define data_write_wait()      while (data_transmitting()) sleep_cpu()
#define data_write(buf,n) {  /* Set transmit buffer (n bytes or string if n=0) and start transmitter. */\
	data_write_wait();\
	data_write_buf = (byte*)buf;\
	data_write_i = n;\
	data_write_o = 0;\
	data_transmitter_start();\
}

// ADC control
#define adc_Hz                F_CPU/128 /* 125000 ticks/s. */
#define adc_MHz               adc_Hz/1000000 /* 1/4 ticks/μs. */
#define adc_diff_settle_us    125 /* 125μs to stabilize differential net. */
#define adc_first_sample_us   27/2/adc_MHz /* 54μs first sample&hold. */
#define adc_first_convert_us  25/adc_MHz /* 100μs first conversion. */
#define adc_sample_us         3/2/adc_MHz  /* 6μs sample&hold. */
#define adc_convert_us        13/adc_MHz /* 52μs conversion. */
#define adc_on()              ADCSRA = B10000111 /* Power on ADC. */
#define adc_isdiff(ch)  (     /* True if the channel is differential. */\
	ch >= B001000 && ch <= B011101) || (ch >= B101000 && ch <= B111101)
#define adc_connect(ch) {     /* Set ADC channel. */\
	ADMUX = B11100000 | ch; ADCSRB = (ch & B100000) >> 2; } 
#define adc_connection() (    /* Get ADC channel. */\
	ADMUX & B1111 | (ADCSRB & B1000) << 2)
#define adc_start()           ADCSRA = B11001111 /* Start ADC. */
#define adc_start_sleep()     sleep_cpu() /* Start ADC in sleep mode. */
#define adc_converting()      (ADCSRA & B00100000) /* Nonzero while the ADC is converting. */
#define adc_stop()            ADCSRA = B00000111 /* Stop ADC. */
#define adc_off() {           /* Power off ADC. */\
	adc_stop(); PRR0 = B10101101; } 

// Sensor control
#define sensor_setup() {      /* Configure sensor pins. */\
    PORTK = 0; /* Configure all pins as either GND or floating.*/\
    DDRK = B00000001; /* Connect sensor ref (analog input pin 8) to GND.*/\
}
#define sensor_connect(n) { /* Set bias (number of pullup resistors to use). */\
    PORTK = B11111111 << (8 - (n));\
}
#define sensor_pullups        4 /* Number of pullups. */
#define sensor_biasR          36000 /* 36kOhm pullup resistance */
#define sensor_biasV          5  /* 5V pullup voltage */
#define sensor_refCh          B100001 /* ADC9 */
#define sensor_nCh            B100010 /* ADC10 */
#define sensor_pCh            B100011 /* ADC11 */
#define sensor_biasCh         B100100 /* ADC12 */
#define sensor_x1Ch           B111011 /* ADC11-ADC10 1× */
#define sensor_x10Ch          B101101 /* ADC11-ADC10 10× */
#define sensor_x200Ch         B101111 /* ADC11-ADC10 200× */
#define sensor_0x1Ch          B111010 /* ADC10-ADC10 1× */
#define sensor_0x10Ch         B101100 /* ADC10-ADC10 10× */
#define sensor_0x200Ch        B101110 /* ADC10-ADC10 200× */

// Heater control
#define heater_setup()  {     /* Enable PWM on digital pins 44-46.*/\
    TCCR5A = B11111100;\
	TCCR5B = B00010001;\
}
#define heater_range          ICR5     /* DAC range for digital pins 44-46. */
#define heater_value          OCR5A=OCR5B=OCR5C /* DAC value for -"-. */
#define heater_trace_vect     TIMER5_COMPA_vect /* Heater on/off event. */
#define heater_trace_on()     TIMSK5 |= B0000010 /* Enable heater tracing. */
#define heater_trace_off()    TIMSK5 &= B1111101 /* Disable heater tracing. */
#define heater_on()           DDRL |= B00111000 /* Enable digital pins 44-46. */
#define heater_off()          DDRL &= B11000111 /* Disable digital pins 44-46.*/
#define heater_set(value, range) { /* Set heater value and range. */\
    /* word value, range; */\
    heater_off();\
    heater_range = range;\
    heater_value = value;\
    heater_on();\
}

// Schedule control
#define schedule_setup() {    /* Configure schedule timing using timer4. */\
    TCCR4A = B11000000;           /* Alarm A on compare match. */\
    TCCR4B = B00000010;           /* Clock at 16MHz/8 = 2MHz. */\
}
#define schedule_start() {    /* Start scheduler. */\
    TIMSK4 = B0000010;            /* Enable alarm A. */\
	OCR4A = TCNT4;                /* Alarm A = now. */\
}
#define schedule_stop()       TIMSK4 = B0000000 /* Disable alarm. */
#define schedule_set(t)       OCR4A = t /* Set alarm time. */
#define schedule_vect         TIMER4_COMPA_vect /* Alarm event. */
#define schedule_time()       TCNT4 /* Time/500ns (word). */
#define schedule_ticks        65536 /* Range 32768μs/500ns.*/
#define schedule_Hz           F_CPU/8 /* 2000000 ticks/s.*/ 
#define schedule_kHz          schedule_Hz/1000 /* 2000 ticks/ms.*/ 
#define schedule_MHz          schedule_kHz/1000 /* 2 ticks/μs.*/ 


/* APPLICATION *****************************************************************/

#define NormChannels          4 /* Normal ADC channels: ref, n, p, bias. */
#define DiffChannels          2 /* Differential ADC channels: pn, nn. */
#define Channels              (NormChannels + DiffChannels)
typedef struct {int time; int value;} Sample;
volatile Sample data[Channels]; // Data buffer.
volatile char samples = 0;    // Current sample.
volatile char task = -1;      // Current task handled by the scheduler.
volatile char iter = -1;      // Iteration counter.
volatile char event = -1;     // Event counter.
volatile char bias = -1;      // Bias counter.
#define TrxTicks              trx_us*schedule_MHz /* 260 ticks/trx */
#define TxSamplesTicks        Channels*3*TrxTicks /* 4680 ticks/txsamples. */
#define FirstTicks            /* 154μs*2MHz = 308 ticks/firstsample. */\
    (adc_first_sample_us + adc_first_convert_us)*schedule_MHz
#define NormTicks             /* 58μs*2MHz = 116 ticks/normsample. */\
    (adc_sample_us + adc_convert_us)*schedule_MHz
#define DiffTicks             /* 279μs*2MHz = 558 ticks/diffsample. */\
    (adc_diff_settle_us + adc_first_sample_us + adc_first_convert_us)*schedule_MHz
#define MeasureTicks          /* 308 + 3*116 + 2*558 = 1772 ticks/acquisition. */\
    (FirstTicks + (NormChannels-1)*NormTicks + DiffChannels*DiffTicks)
#define MinTaskTicks          /* >6452 ticks/task (3226μs). */\
    (TxSamplesTicks + MeasureTicks)
#define adc_convert(condition)/* Start and wait for ADC, sleep while condition(). */\
    for (sleep_cpu(); adc_converting(); if (t() - t1 < t2) sleep_cpu())

ISR (ADC_vect) {
    /* On adc completion events after adc_start() or adc_start_sleep() commands.
	Store adc values in data buffer, switch off adc and transmit data.
	*/
	data[samples++].value = ADC;
	if (samples==Channels) {
	    adc_off();
		samples = 0;
	    data_write(data, Channels*sizeof(Sample));
	}
}
void measure(word t0, dt;) {
    data[samples].time = ((schedule_time() - t0)/schedule_MHz); // µs offset from t0
	byte samples0 = samples; // ADC_vect will update this.
	adc_start_sleep(); // Will return after any ISR is ready, not only ADC_vect.
	while(samples == samples0) cpu_sleep(); // Wait in sleep mode until ADC_vect is ready.
}
ISR (schedule_vect) {
    /* On schedule_time() == t events after schedule_set(t).
	Alternate between 10 iterations of two heat settings: 1s*5V and 1s*1.4V. 
    Send samples acquired in previous task.
    Alternate through five bias settings: oo, 36kOhm, 18kOhm, 12kOhm and 9kOhm.
    Measure six channels: ref, n, p, bias, nnx1 and pnx1.
    */
    static word skip=0; // Skip up to 2^16*schedule_ticks/schedule_Hz > 35 minutes.
    if (skip)
        skip--;
    else {
        static word alarm; // Alarm tick.
        static word endtime; // End-of-task tick.
        long ticks; // Ticks to next task.
		switch (++event) {
            case 0: // Start by reading task to start with.
			    data_read(&task, 1); // Start reading a task.
            case 1: // Alternate between 10 iterations of 2 heater voltages.
			    data_read(&task, 1); // Wait and start reading next task.
                alarm = schedule_time(); // Assume 
			    switch(task) {
                    case 0: // 1s * 5V
                        heater_set(25, 25);
                        ticks = schedule_Hz;
                        break;
                    case 1: // 1s * 1.4V
                        heater_set(7, 25);
                        ticks = schedule_Hz;
                        break;
					default:
					    ticks = MinTaskTicks;
					    break;
                }
				endtime = alarm + ticks;
                if (samples) 
				    tx_start();
                schedule_set(alarm = endtime - MeasureTicks);
                skip = ticks / schedule_ticks;
                break;
            case 2: // Alternate through bias 0-4. Measure ref, n, p, bias. Settle calibration. */
			    tx_stop();
                sensor_set_bias(bias = ++bias % 5);
                adc_on();
                adc_set_channel(sensor_refCh);
                timestamp();
                adc_convert(schedule_time, schedule_time() + FirstTicks, 100);
                adc_set_channel(sensor_nCh);
                timestamp();
                adc_start_sleep();
                adc_set_channel(sensor_pCh);
                timestamp();
                adc_start_sleep();
                adc_set_channel(sensor_biasCh);
                timestamp();
                adc_start_sleep();
                adc_off();
                adc_set_channel(sensor_0x1Ch);
                schedule_set(alarm = endtime - 2 * DiffTicks);
                break;
            case 3: // Convert calibration. Settle sensor signal. */
                adc_on();
                timestamp();
                adc_start_sleep();
                adc_off();
                adc_set_channel(sensor_x1Ch);
                schedule_set(alarm = endtime - DiffTicks);
                break;
            case 4: // Convert sensor signal. Finish heat control . */
                schedule_set(alarm = endtime);
                event = 0; // Next event = 1.
                adc_on();
                timestamp();
                adc_start_sleep();
                adc_off();
                break;
        }
    }
}

void setup() {
  noInterrupts();
  power_setup();
  sensor_setup();
  heater_setup();
  schedule_setup();
  data_setup();
  server_setup();
  interrupts();
  
  data_write("http://");
  data_write(server_address()); // 7 + len("128.128.128.128") = 23 bytes
  data_transmitter_start();
  data_receiver_start();
}


/* STATUS **********************************************************************/

ISR(BADISR_vect) {
    /* On events that has no handler due to a programming bug.
    */
}

void loop() {
    static int hits = 0;
    EthernetClient client = status.available();
    if (client) {
        while (client.connected())
            if (client.available()) {
                client.println(F("HTTP/1.1 200 OK"));
                client.println(F("Content-Type: text/html"));
                client.println();
                client.println(hits);
                client.println(task);
                client.println(iter);
                client.println(event);
                client.println(bias);
                break;
            }
        client.stop();
    }
}

/* MEMORY USAGE ***************************************************************
macen:/tmp/build5040604529806027944.tmp joakim$ /Applications/Arduino.app/Contents/Resources/Java/hardware/tools/avr/bin/avr-size -C --mcu=atmega328p debug_signal_strength_on_weather_station.cpp.elf
AVR Memory Usage
----------------
Device: atmega328p

Program:    4902 bytes (15.0% Full)
(.text + .data + .bootloader)

Data:       1255 bytes (61.3% Full)
(.data + .bss + .noinit)
*/