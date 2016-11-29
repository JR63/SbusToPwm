/*

SBus to PWM converter


Massive refactoring (c) by Jörg-D. Rothfuchs


Refactoring based on this code:
	https://github.com/MikeBland/SbusToPpm


Collection of helpful links:
	https://developer.mbed.org/users/Digixx/notebook/futaba-s-bus-controlled-by-mbed/
	https://github.com/sebseb7/SbusToPPM


The protocol is 25 Byte long and is send every 14ms (analog mode) or 7ms (highspeed mode).

One Byte = 1 startbit + 8 databit + 1 paritybit + 2 stopbit (8E2), baudrate = 100'000 bit/s, so one bit takes 10 microseconds.

The highest bit is send first, so data "00000000001" = 1024.

The logic is inverted out of the receiver.

[startbyte] [data1] [data2] .... [data22] [flags] [endbyte]

startbyte = 11110000b (0xF0)

data 1-22
	[ch1, 11bit] [ch2, 11bit] .... [ch16, 11bit] (ch# = 0 bis 2047) channel 1 uses 8 bits from data1 and 3 bits from data2 channel 2 uses last 5 bits from data2 and 6 bits from data3 and so on.

flags:
	bit7 = ch17 = digital channel			(0x80)
	bit6 = ch18 = digital channel			(0x40)
	bit5 = Frame lost, (red LED on receiver)	(0x20)
	bit4 = failsafe activated			(0x10)
	bit3 = n/a					(0x08)
	bit2 = n/a					(0x04)
	bit1 = n/a					(0x02)
	bit0 = n/a					(0x01)

endbyte = 00000000b (Prior to the s.bus2 capable receivers, and the R7003SB)

For Futaba FASSTest modes there are other endbytes to consider (highest bit first):

"FASSTest 18CH" mode
00100000	0x04
00101000	0x14
00100100	0x24
00101100	0x34

"FASSTest 12CH" mode
00010000	0x08


Inverter:
	NPN SOT-23		evtl.	http://www.reichelt.de/BC-Transistoren/BC-818-16-SMD/3/index.html?ACTION=3&LA=446&ARTICLE=18556&GROUPID=7206&artnr=BC+818-16+SMD&SEARCH=NPN%2B%2BSOT-23
	10k SMD
	4k7 SMD


AD4 -> GND	set current values to failsave

maybe later, but needs external pull up
AD6 -> GND	set current values to failsave

*/


#include <avr/eeprom.h>


//#define DEBUG

#define SIMULATE_SBUS


// Hardware pin mapping
#define DDR_IO2			DDRD
#define PORT_IO2		PORTD
#define PIN_IO2			PIND
#define BIT_IO2			2
#define DDR_IO3			DDRD
#define PORT_IO3		PORTD
#define PIN_IO3			PIND
#define BIT_IO3			3
#define DDR_IO4			DDRD
#define PORT_IO4		PORTD
#define PIN_IO4			PIND
#define BIT_IO4			4
#define DDR_IO5			DDRD
#define PORT_IO5		PORTD
#define PIN_IO5			PIND
#define BIT_IO5			5
#define DDR_IO6			DDRD
#define PORT_IO6		PORTD
#define PIN_IO6			PIND
#define BIT_IO6			6
#define DDR_IO7			DDRD
#define PORT_IO7		PORTD
#define PIN_IO7			PIND
#define BIT_IO7			7

#define DDR_IO8			DDRB
#define PORT_IO8		PORTB
#define PIN_IO8			PINB
#define BIT_IO8			0
#define DDR_IO9			DDRB
#define PORT_IO9		PORTB
#define PIN_IO9			PINB
#define BIT_IO9			1
#define DDR_IO10		DDRB
#define PORT_IO10		PORTB
#define PIN_IO10		PINB
#define BIT_IO10		2
#define DDR_IO11		DDRB
#define PORT_IO11		PORTB
#define PIN_IO11		PINB
#define BIT_IO11		3
#define DDR_IO12		DDRB
#define PORT_IO12		PORTB
#define PIN_IO12		PINB
#define BIT_IO12		4
#define DDR_IO13		DDRB
#define PORT_IO13		PORTB
#define PIN_IO13		PINB
#define BIT_IO13		5

#define DDR_ADC0		DDRC
#define PORT_ADC0		PORTC
#define PIN_ADC0		PINC
#define BIT_ADC0		0
#define DDR_ADC1		DDRC
#define PORT_ADC1		PORTC
#define PIN_ADC1		PINC
#define BIT_ADC1		1
#define DDR_ADC2		DDRC
#define PORT_ADC2		PORTC
#define PIN_ADC2		PINC
#define BIT_ADC2		2
#define DDR_ADC3		DDRC
#define PORT_ADC3		PORTC
#define PIN_ADC3		PINC
#define BIT_ADC3		3
#define DDR_ADC4		DDRC
#define PORT_ADC4		PORTC
#define PIN_ADC4		PINC
#define BIT_ADC4		4
#define DDR_ADC5		DDRC
#define PORT_ADC5		PORTC
#define PIN_ADC5		PINC
#define BIT_ADC5		5


#if F_CPU == 16000000L 		// 16 MHz clock
#define PULSE_SCALE		16
#define BAUD_57600		16
#define BAUD_100000		9
#elif F_CPU == 8000000L		//  8 MHz clock
#define PULSE_SCALE		8
#define BAUD_57600		8
#define BAUD_100000		4
#endif

#define TIME_LONG		(2500 * PULSE_SCALE)
#define T_MS_3000		(3000 * PULSE_SCALE)
#define T_MS_500		( 500 * PULSE_SCALE)
#define DELAY_CALC		( 150 * PULSE_SCALE)
#define DELAY_OFFSET		(  20 * PULSE_SCALE)

#define PULSE_MIN		 800
#define PULSE_MAX		2200

#define IDLE			0	// idle
#define PULSING			1	// generating PWM pulses


#define SBUS_START_BYTE_VALUE	0x0F
#define SBUS_START_BYTE_INDEX	0


/*
For SBUS_END_BYTE_VALUE mention different endbytes by Futaba FASSTest modes (highest bit first):

"FASSTest 18CH" mode
00100000	0x04
00101000	0x14
00100100	0x24
00101100	0x34

"FASSTest 12CH" mode
00010000	0x08
*/

#define SBUS_END_BYTE_VALUE	0x00
#define SBUS_END_BYTE_INDEX	24

#define SBUS_CH_17_MASK		0x80		// TODO or 0x01
#define SBUS_CH_18_MASK		0x40		// TODO or 0x02

#define SBUS_PACKET_LEN		25

#define SBUS_BUF_LEN		28
#define SBUS_BUF_LEN_LAST	27


#if 1
#define PULSE_PACKET_ROUNDS	4
#define PULSE_PACKET_CNT	4
#define PULSE_PACKET_CNT_BY_2	8
#else
#define PULSE_PACKET_ROUNDS	3
#define PULSE_PACKET_CNT	6
#define PULSE_PACKET_CNT_BY_2	12
#endif


#define CHANNEL_17_INDEX	16
#define CHANNEL_18_INDEX	17
#define NUMBER_CHANNELS		18


#define PULSE_SET_OFFSET	(PULSE_MAX + PULSE_PACKET_CNT * (DELAY_OFFSET / PULSE_SCALE))


#define ENABLE_TIMER_INTERRUPT()	(TIMSK1 |= (1 << OCIE1A))
#define DISABLE_TIMER_INTERRUPT()	(TIMSK1 &= ~(1 << OCIE1A))
#define CLEAR_TIMER_INTERRUPT()		(TIFR1 = (1 << OCF1A))


struct t_pulses {
	uint8_t *port;
	uint8_t bit;
	uint16_t fire;
} Pulses[PULSE_PACKET_CNT_BY_2];


uint8_t* Ports[NUMBER_CHANNELS] = {
	(uint8_t*) &PORT_IO2,
	(uint8_t*) &PORT_IO3,
	(uint8_t*) &PORT_IO4,
	(uint8_t*) &PORT_IO5,
	(uint8_t*) &PORT_IO6,
	(uint8_t*) &PORT_IO7,
	(uint8_t*) &PORT_IO8,
	(uint8_t*) &PORT_IO9,
	(uint8_t*) &PORT_IO10,
	(uint8_t*) &PORT_IO11,
	(uint8_t*) &PORT_IO12,
	(uint8_t*) &PORT_IO13,
	(uint8_t*) &PORT_ADC0,
	(uint8_t*) &PORT_ADC1,
	(uint8_t*) &PORT_ADC2,
	(uint8_t*) &PORT_ADC3,
	(uint8_t*) &PORT_ADC4,
	(uint8_t*) &PORT_ADC5
};

uint8_t Bits[NUMBER_CHANNELS] = {
	1 << BIT_IO2,
	1 << BIT_IO3,
	1 << BIT_IO4,
	1 << BIT_IO5,
	1 << BIT_IO6,
	1 << BIT_IO7,
	1 << BIT_IO8,
	1 << BIT_IO9,
	1 << BIT_IO10,
	1 << BIT_IO11,
	1 << BIT_IO12,
	1 << BIT_IO13,
	1 << BIT_ADC0,
	1 << BIT_ADC1,
	1 << BIT_ADC2,
	1 << BIT_ADC3,
	1 << BIT_ADC4,
	1 << BIT_ADC5
};


static volatile uint8_t PulseOutState = IDLE;

static uint8_t PulsesIndex = 0;

static uint16_t FailsafeTimes[NUMBER_CHANNELS];
static uint16_t PulseTimes[NUMBER_CHANNELS];

static uint8_t SBusBuffer[SBUS_BUF_LEN] = {0};
static uint8_t SBusIndex = 0;

static uint32_t LastSBusReceived = 0;

static uint8_t SerialMode = 1;




ISR(TIMER1_COMPA_vect)
{
	if (PulsesIndex < PULSE_PACKET_CNT_BY_2) {
		t_pulses *p = &Pulses[PulsesIndex];
		OCR1A = p->fire;			// next shot
		if (PulsesIndex++ < PULSE_PACKET_CNT)	// if start
			*p->port |= p->bit;		//   set the output
		else					// if stop
			*p->port &= p->bit;		//   clear the output
	}
	
	if (PulsesIndex >= PULSE_PACKET_CNT_BY_2) {
		DISABLE_TIMER_INTERRUPT();		// pulses are finished
		PulseOutState = IDLE;
	}
}


// replacement of millis() and micros(), polling, no interrupts
// micros() MUST be called at least once every 4 milliseconds

uint32_t TotalMillis;	// TODO

uint32_t micros()
{
	static uint16_t MillisPrecount = 0;
	static uint16_t lastTimerValue = 0;
	static uint32_t TotalMicros = 0;
	static uint8_t Correction = 0;

	uint16_t elapsed;
	uint8_t millisToAdd;
	uint8_t oldSREG = SREG;
	
	cli();
	uint16_t time = TCNT1;	// Read timer 1
	SREG = oldSREG;

	elapsed = time - lastTimerValue;
	elapsed += Correction;
#if F_CPU == 16000000L  // 16MHz clock                                                  
	Correction = elapsed & 0x0F;
	elapsed >>= 4;
#elif F_CPU == 8000000L   // 8MHz clock
	Correction = elapsed & 0x07;
	elapsed >>= 3;
#endif		
	
	uint32_t ltime = TotalMicros;
	ltime += elapsed;
	cli();
	TotalMicros = ltime;		// done this way for RPM to work correctly
	lastTimerValue = time;
	SREG = oldSREG;			// still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0;
	
#if F_CPU == 8000000L			// 8MHz clock
	if (elapsed > 3999) {
		millisToAdd = 4;
		elapsed -= 4000;
	}
#endif		
	if (elapsed > 3999) {
		millisToAdd += 4;
		elapsed -= 4000;
	} else if (elapsed > 2999) {
		millisToAdd += 3;		
		elapsed -= 3000;
	} else if (elapsed > 1999) {
		millisToAdd += 2;
		elapsed -= 2000;
	} else if (elapsed > 999) {
		millisToAdd++;
		elapsed -= 1000;
	}
	
	TotalMillis += millisToAdd;
	MillisPrecount = elapsed;
	
	return TotalMicros;
}


uint32_t millis()
{
	micros();
	return TotalMillis;
}


void eeprom_write_byte_cmp(uint8_t dat, uint16_t pointer_eeprom)
{
  while(EECR & (1 << EEPE)) {}		// wait for EEPROM is ready
  
  EEAR = pointer_eeprom;
  EECR |= 1 << EERE;
  
  if (dat == EEDR) return;

  EEDR = dat;
  uint8_t flags = SREG;
  cli();
  EECR |= 1 << EEMPE;
  EECR |= 1 << EEPE;
  SREG = flags;
}


void eeWriteBlockCmp(const void *i_pointer_ram, uint16_t i_pointer_eeprom, size_t size)
{
  const char* pointer_ram = (const char*)i_pointer_ram;
  uint16_t pointer_eeprom = i_pointer_eeprom;
  
  while(size) {
    eeprom_write_byte_cmp(*pointer_ram++, pointer_eeprom++);
    size -= 1;
  }
}


void writeFailsafe()
{
  eeWriteBlockCmp((const void*)FailsafeTimes, (uint16_t)0, 32);
}


void readFailsafe()
{
  eeprom_read_block(FailsafeTimes, (const void*)0, 32);
}


// mode 0 -> serial  57600 baud
// mode 1 -> serial 100000 baud
void setSerialMode(uint8_t mode)
{
	if (mode == 0) {		// 57600 baud                                                
		UBRR0L = BAUD_57600;
		UCSR0C = (1 << UCSZ00) | (1 << UCSZ01 );
	} else {			// 100000 baud                                                
		UBRR0L = BAUD_100000;
		UCSR0C = (1 << UCSZ00) | (1 << UCSZ01 ) | (1 << UPM01);
		UCSR0B &= ~TXEN0;
	}
	SerialMode = mode;
}


void enterFailsafe()
{
	uint8_t i;
	for (i = 0; i < NUMBER_CHANNELS; i++) {
#if 0
		PulseTimes[i] = FailsafeTimes[i];
#else
#ifdef SIMULATE_SBUS
		PulseTimes[i] = 1500;				// TODO test pattern
#else
		PulseTimes[i] = PULSE_MIN + i * 85;		// TODO test pattern
#endif
#endif
	}
}


void initFailsave()
{
	uint8_t i;
	uint8_t updateFailsafe = 0;
	
	readFailsafe();
	
	for (i = 0; i < NUMBER_CHANNELS; i++) {
		if ((FailsafeTimes[i] < PULSE_MIN ) || (FailsafeTimes[i] > PULSE_MAX)) {
			FailsafeTimes[i] = 1500;
			updateFailsafe = 1;
		}
	}
	if (updateFailsafe)
		writeFailsafe();

	enterFailsafe();
}


static void initUart()
{
	UBRR0H = 0;
	setSerialMode(1);		// 1 -> serial 100000
	UCSR0A = 0;
	PORTD |= 0x03;			// enable pullup
	UCSR0B = (1 << RXEN0);		// enable receiver
}


void initBasic()
{
	// init Timer1
	TCCR1A = 0x00;			// init
	TCCR1B = 0xC1;			// i/p noise cancel, rising edge, clock / 1
	TIFR1 = (1 << OCF1A);
	TIMSK1 |= (1 << OCIE1A);
	
	// set outputs low
	PORTB  = 0x00;
	PORTC &= 0xF0;
	PORTD &= 0x03;
	
	// set as output
	DDR_IO2  |= (1 << BIT_IO2);
	DDR_IO3  |= (1 << BIT_IO3);
	DDR_IO4  |= (1 << BIT_IO4);
	DDR_IO5  |= (1 << BIT_IO5);
	DDR_IO6  |= (1 << BIT_IO6);
	DDR_IO7  |= (1 << BIT_IO7);
	DDR_IO8  |= (1 << BIT_IO8);
	DDR_IO9  |= (1 << BIT_IO9);
	DDR_IO10 |= (1 << BIT_IO10);
	DDR_IO11 |= (1 << BIT_IO11);
	DDR_IO12 |= (1 << BIT_IO12);
	DDR_IO13 |= (1 << BIT_IO13);
	DDR_ADC0 |= (1 << BIT_ADC0);
	DDR_ADC1 |= (1 << BIT_ADC1);
	DDR_ADC2 |= (1 << BIT_ADC2);
	DDR_ADC3 |= (1 << BIT_ADC3);
	
	// AD4, AD5 digital input with pullup
	DDRC &= ~0x30;
	PORTC |= 0x30;

	// PD1 ~ TX
	DDRD &= ~0x02;
	PORTD |= 0x02;

#ifdef DEBUG
	DDRC |= 0x20;
	PORTC &= ~0x20;
#endif
}


void setup(void)
{	
	initBasic();
	initUart();
	initFailsave();

	sei();
}


// TODO
void checkFailsafePin()
{
	static uint32_t LastPinTime = 0;
	static uint8_t LastPinState = 0;

	if (millis() - LastPinTime < 500)
		return;
		
	if (( PINC & 0x10 ) == 0) {			// AD4 to GND ?
		if (LastPinState == 0) {
			LastPinTime = millis();
			LastPinState = 1;
			uint8_t i;
			for (i = 0; i < NUMBER_CHANNELS; i++)
				FailsafeTimes[i] = PulseTimes[i];
			writeFailsafe();
		}
	} else {
		if (LastPinState) {
			LastPinState = 0;
			LastPinTime = millis();
		}
	}
}


void readSBus()
{
	uint8_t sbus_byte;
	
	while (UCSR0A & (1 << RXC0)) {
		sbus_byte = UDR0;
		if (SBusIndex || (sbus_byte == SBUS_START_BYTE_VALUE)) {	// if data byte or startbyte
			SBusBuffer[SBusIndex] = sbus_byte;
			if (SBusIndex < SBUS_BUF_LEN_LAST)
				SBusIndex++;
		}
	}
}


static uint8_t processSBusFrame()
{
#ifdef SIMULATE_SBUS

#define SIMULATE_STEP_WIDTH	1

	static int8_t off[NUMBER_CHANNELS] = {+1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1, +1, -1};
	static uint32_t last_processed = 0;
	uint8_t i;
	
	if ((micros() - last_processed) > 20000) {	
		last_processed = micros();
		for (i = 0; i < NUMBER_CHANNELS; i++) {
			if (off[i] == 1) {
				if (PulseTimes[i] >= PULSE_MAX)
					off[i] = -1;
			} else {
				if (PulseTimes[i] <= PULSE_MIN)
					off[i] = +1;
			}
			PulseTimes[i] = PulseTimes[i] + (off[i] * SIMULATE_STEP_WIDTH);
		}
	}
#else	// SIMULATE_SBUS
	uint8_t inputbitsavailable = 0;
	uint8_t i;
	uint32_t inputbits = 0;
	uint8_t *sbus = SBusBuffer;
	uint16_t pulse;

	if (SBusIndex < SBUS_PACKET_LEN)
		return 0;

	// TODO mask the FASSTest bits for working with Futaba "FASSTest 18/12CH" mode
	if (sbus[SBUS_END_BYTE_INDEX] != SBUS_END_BYTE_VALUE) {
		SBusIndex = 0;
		return 0;		// not a valid SBUS frame
	}

	if (*sbus++ != SBUS_START_BYTE_VALUE) {
		SBusIndex = 0;
		return 0;		// not a valid SBUS frame
	}

#ifdef DEBUG
	PORTC ^= 0x20;
#endif

	for (i = 0; i < NUMBER_CHANNELS - 2; i++) {
		while (inputbitsavailable < 11) {
			inputbits |= (uint32_t)*sbus++ << inputbitsavailable;
			inputbitsavailable += 8;
		}
		pulse = ((int16_t)(inputbits & 0x7FF) - 0x3E0) * 5 / 8 + 1500;
		if ((pulse > PULSE_MIN) && (pulse < PULSE_MAX))
			PulseTimes[i] = pulse;
			
		inputbitsavailable -= 11;
		inputbits >>= 11;
	}

	if (*sbus & SBUS_CH_17_MASK)
		PulseTimes[CHANNEL_17_INDEX] = 2200;
	else
		PulseTimes[CHANNEL_17_INDEX] =  800;
		
	if (*sbus & SBUS_CH_18_MASK)
		PulseTimes[CHANNEL_18_INDEX] = 2200;
	else
		PulseTimes[CHANNEL_18_INDEX] =  800;
#endif	// SIMULATE_SBUS

	LastSBusReceived = millis();
	SBusIndex = 0;
	
	return 1;
}


void setPulses(uint16_t* times, uint8_t k, uint8_t n, uint16_t time)
{
	uint16_t m;
	uint8_t i;
	uint8_t j = 0;
	
	m = times[j];
	for (i = 1; i < PULSE_PACKET_CNT; i++) {	// find next shortest pulse
		if (times[i] < m) {			// if this one is shorter
			j = i;
			m = times[j];
		}
	}
	times[j] = 0xFFFF;				// mark as done
	j += k;						// add offset
	
	Pulses[n].port = Ports[j];			// set the port for this pulse
	Pulses[n + PULSE_PACKET_CNT].port = Ports[j];	// set the port for this pulse

	Pulses[n].bit = Bits[j];			// set the bit mask for this pulse on
	Pulses[n + PULSE_PACKET_CNT].bit = ~Bits[j];	// set the bit mask for this pulse off

	Pulses[n + PULSE_PACKET_CNT - 1].fire = time + DELAY_OFFSET * n + m * PULSE_SCALE;
	if (n < (PULSE_PACKET_CNT - 1)) {
		Pulses[n].fire = time + DELAY_OFFSET * (n+1);
	} else {
		Pulses[n + PULSE_PACKET_CNT].fire = time + TIME_LONG;
	}
	
	readSBus();
}


void setPulseTimes(uint8_t PulseStateMachine)
{
	uint16_t *pulsePtr = PulseTimes;
	uint16_t times[PULSE_PACKET_CNT];
	uint16_t time;
	uint8_t i;
	uint8_t k;					// offset into Ports and Bits
	
	if (PulseStateMachine == PULSE_PACKET_ROUNDS)
		return;
	
	DISABLE_TIMER_INTERRUPT();
	
	i = PulseStateMachine * PULSE_PACKET_CNT;
	pulsePtr += i;
	k = i;
	
	for (i = 0; i < PULSE_PACKET_CNT; i++)
		times[i] = pulsePtr[i];			// local copy of pulses to process
	
	cli();
	time = TCNT1 + DELAY_CALC;			// start the pulses after calculation
	sei();						// gives time for this code to finish
	
	for (i = 0; i < PULSE_PACKET_CNT; i++)
		setPulses(times, k, i, time);		// set the pulses
		
	cli();
	OCR1A = time;					// set for first interrupt
	sei();						// gives time for this code to finish
	
	CLEAR_TIMER_INTERRUPT();			// clear flag in case it is set
	PulsesIndex = 0;				// start here
	PulseOutState = PULSING;
	ENABLE_TIMER_INTERRUPT();			// allow interrupt to run
}


int main(void)
{
	uint32_t Last16ChannelsStartTime = 0;
	uint32_t Last04ChannelsStartTime = 0;
	uint8_t PulseStateMachine = PULSE_PACKET_ROUNDS;

	setup();
	
	while (1) {
		readSBus();
		
#ifdef SIMULATE_SBUS
		processSBusFrame();
#else
		if (SBusIndex >= SBUS_PACKET_LEN)
			processSBusFrame();
#endif
		
		readSBus();
		
		if ((micros() - Last16ChannelsStartTime) > 20000) {		// time for the first pulse set
			if (PulseStateMachine == PULSE_PACKET_ROUNDS) {
				PulseStateMachine = 0;
				setPulseTimes(PulseStateMachine++);		// first pulse set
				Last16ChannelsStartTime = micros();
				Last04ChannelsStartTime = micros();
			}
		}
		
		readSBus();
		
		if ((micros() - Last04ChannelsStartTime) > PULSE_SET_OFFSET) {	// time for the next pulse set
			if (PulseStateMachine < PULSE_PACKET_ROUNDS) {
				setPulseTimes(PulseStateMachine++);		// next pulse set
				Last04ChannelsStartTime = micros();
			}
		}
		
		readSBus();
		
		if ((millis() - LastSBusReceived) > 500)
			enterFailsafe();
		
		checkFailsafePin();
	}
}
