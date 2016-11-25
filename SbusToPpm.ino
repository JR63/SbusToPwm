/*

SBus to PPM converter


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
AD5 -> GND	channel 1-8 and 9-16 are swapped
TX  -> GND	via 1k only 8 channel every 9 ms

maybe later, but needs external pull up
AD6 -> GND	only 8 channel every 9 ms
AD7 -> GND	unused yet

*/


#include <avr/eeprom.h>


//#define DEBUG


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


#if F_CPU == 16000000L  // 16MHz clock
#define DELAY150		2400
#define DELAY20			320
#define DELAY40			640
#define DELAY60			960
#define TIME_LONG		40000
#define PULSE_SCALE		16
#elif F_CPU == 8000000L   // 8MHz clock
#define DELAY150		1200
#define DELAY20			160
#define DELAY40			320
#define DELAY60			480
#define TIME_LONG		20000
#define PULSE_SCALE		8
#endif


#define IDLE			0	// idle
#define PULSING			1	// generating PPM pulses


#define	ONE_TO_FOUR		0
#define	FIVE_TO_EIGHT		1
#define	NINE_TO_TWELVE		2
#define	THIRTEEN_TO_SIXTEEN	3
#define	END_PULSES		4


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

#define SBUS_PACKET_LEN		25

#define SBUS_BUF_LEN		28
#define SBUS_BUF_LEN_LAST	27


#define NUMBER_CHANNELS		16


#define ENABLE_TIMER_INTERRUPT()	(TIMSK1 |= (1 << OCIE1A))
#define DISABLE_TIMER_INTERRUPT()	(TIMSK1 &= ~(1 << OCIE1A))
#define CLEAR_TIMER_INTERRUPT()		(TIFR1 = (1 << OCF1A))


struct t_pulses {
	uint8_t *port;
	uint8_t bit;	
	uint8_t start;
	uint16_t nextTime;
} Pulses[8];


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
	(uint8_t*) &PORT_ADC3
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
	1 << BIT_ADC3
};


static volatile uint16_t LastRcv = 0;

static volatile uint8_t PulseOutState = IDLE;

static uint8_t PulsesIndex = 0;

static uint16_t FailsafeTimes[NUMBER_CHANNELS];
static uint16_t PulseTimes[NUMBER_CHANNELS];

static uint8_t EightOnly = 0;
static uint8_t ChannelSwap = 0;

static uint8_t SBusBuffer[SBUS_BUF_LEN] = {0};
static uint8_t SBusIndex = 0;

static uint32_t LastSBusReceived = 0;
static uint8_t SBusHasBeenReceived = 0;

static uint8_t SerialMode = 1;




ISR(TIMER1_COMPA_vect)
{			
	if (PulseOutState == PULSING) {
		if (PulsesIndex < 8) {
			t_pulses *p = &Pulses[PulsesIndex];
			OCR1A = p->nextTime;		// time of next action
			if (p->start)			// if start
				*p->port |= p->bit;	//   set the output
			else				// if stop
				*p->port &= ~p->bit;	//   clear the output
			PulsesIndex++;			// next entry
		}
		// TODO
		if (PulsesIndex >= 8) {
			DISABLE_TIMER_INTERRUPT();	// 4 pulses are finished
			PulseOutState = IDLE;
		}
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
	if (mode == 0) {		// 57600
#if F_CPU == 16000000L			// 16 MHz clock                                                  
		UBRR0L = 16;		// for 57600 baud, use 9 for 100000 baud
#elif F_CPU == 8000000L			//  8 MHz clock
		UBRR0L = 8;		// for 57600 baud, use 4 for 100000 baud
#endif		
		UCSR0C = (1 << UCSZ00) | (1 << UCSZ01 );
	} else {
#if F_CPU == 16000000L			// 16 MHz clock                                                  
		UBRR0L = 9;		// for 100000 baud
#elif F_CPU == 8000000L			//  8 MHz clock
		UBRR0L = 4;		// for 100000 baud
#endif		
		UCSR0C = (1 << UCSZ00) | (1 << UCSZ01 ) | (1 << UPM01);
		UCSR0B &= ~TXEN0;
	}
	SerialMode = mode;
}


void enterFailsafe()
{
	uint8_t i;
	for (i = 0; i < NUMBER_CHANNELS; i++) {
		PulseTimes[i] = FailsafeTimes[i];
//		PulseTimes[i] = 800 + i * 85;		// TODO test pattern
	}
}


void initFailsave()
{
	uint8_t i;
	uint8_t updateFailsafe = 0;
	
	readFailsafe();
	
	for (i = 0; i < NUMBER_CHANNELS; i++) {
		if ((FailsafeTimes[i] < 800 ) || (FailsafeTimes[i] > 2200)) {
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

	if ((PINC & 0x20) == 0)		// AD5 to GND ?
		ChannelSwap = 1;

	if ((PIND & 0x02) == 0)		// PD1 to GND ?		PD1 ~ TX	TODO check for floating
		EightOnly = 1;

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
			cli();
			LastRcv = TCNT1;
			sei();
			if (SBusIndex < SBUS_BUF_LEN_LAST)
				SBusIndex++;
		}
	}
}


static uint8_t processSBusFrame()
{
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

	LastSBusReceived = millis();
	SBusHasBeenReceived = 1;

	for (i = 0; i < NUMBER_CHANNELS; i++) {
		while (inputbitsavailable < 11) {
			inputbits |= (uint32_t)*sbus++ << inputbitsavailable;
			inputbitsavailable += 8;
		}
		pulse = ((int16_t)(inputbits & 0x7FF) - 0x3E0) * 5 / 8 + 1500;
		if ((pulse > 800) && (pulse < 2200))
			PulseTimes[i] = pulse;
			
		inputbitsavailable -= 11;
		inputbits >>= 11;
	}
	SBusIndex = 0;
	
	return 1;
}


void setPulses(uint16_t* times, uint8_t k, uint8_t n, uint16_t time)
{
	uint16_t m;
	uint8_t i;
	uint8_t j = 0;
	
	m = times[j];
	for (i = 1; i < 4; i++) {			// find next shortest pulse
		if (times[i] < times[i-1]) {		// if this one is shorter
			j = i;
			m = times[j];
		}
	}
	times[j] = 0xFFFF;				// make local copy very large
	j += k;						// add offset
	
	Pulses[n+4].port  = Pulses[n].port = Ports[j];	// set the port for this pulse
	Pulses[n+4].bit   = Pulses[n].bit = Bits[j];	// set the bit for this pulse
	Pulses[n+4].start = 0;				// mark the end
	Pulses[n].start   = 1;				// mark the start
	if (n < 3) {
		Pulses[n].nextTime   = time + DELAY20 * (n+1);
		Pulses[n+3].nextTime = time + DELAY20 * n + m * PULSE_SCALE;
	} else {
		Pulses[n+3].nextTime = time + DELAY60 + m * PULSE_SCALE;
		Pulses[n+4].nextTime = time + TIME_LONG;
	}
	
	readSBus();
}


void setPulseTimes(uint8_t PulseStateMachine)
{
	uint16_t *pulsePtr = PulseTimes;
	uint16_t times[4];
	uint8_t i;
	uint8_t k = 0;					// offset into Ports and Bits
	
	if (PulseStateMachine == FIVE_TO_EIGHT) {
		pulsePtr += 4;				// move on to second 4 pulses
		k = 4;
	} else if (PulseStateMachine == NINE_TO_TWELVE) {
		pulsePtr += 8;				// move on to third 4 pulses
		k = 8;
	} else if (PulseStateMachine == THIRTEEN_TO_SIXTEEN) {
		pulsePtr += 12;				// move on to fourth 4 pulses
		k = 12;
	} else if (PulseStateMachine == END_PULSES) {
		return;
	}

	if (ChannelSwap) {				// swap chanels 1-8 and 9-16
		if (k >= 8)
			k -= 8;
		else
			k += 8;
	}

	for (i = 0; i < 4; i++)
		times[i] = pulsePtr[i];			// local copy of pulses to process
		
	cli();
	uint16_t time = TCNT1 + DELAY150;		// start the pulses in 150 uS
	sei();						// gives time for this code to finish
	
	for (i = 0; i < 4; i++)
		setPulses(times, k, i, time);		// set the pulses
	
	cli();
	OCR1A = time;					// set for first interrupt
	sei();
	
	CLEAR_TIMER_INTERRUPT();			// clear flag in case it is set
	PulsesIndex = 0;				// start here
	PulseOutState = PULSING;
	ENABLE_TIMER_INTERRUPT();			// allow interrupt to run
}


int main(void)
{
	uint8_t PulseStateMachine = ONE_TO_FOUR;
	uint32_t current_micros;
	uint32_t Last16ChannelsStartTime = 0;
	uint32_t Last04ChannelsStartTime = 0;
	uint16_t TCnt;

	setup();
	
	for(;;) {
		readSBus();
		
		cli();
		TCnt = TCNT1;
		sei();

#if F_CPU == 16000000L  // 16MHz clock
		if ((TCnt - LastRcv) > 8000) {
#elif F_CPU == 8000000L   // 8MHz clock
		if ((TCnt - LastRcv) > 4000) {
#endif
			if (SBusIndex >= SBUS_PACKET_LEN) {
				if (processSBusFrame()) {
					if (SerialMode && PulseOutState == IDLE) {
						current_micros = micros();
						uint16_t rate = 17900;
						if (EightOnly)
							rate = 8900;
						if ((current_micros - Last16ChannelsStartTime ) > rate)
							Last16ChannelsStartTime = current_micros - 21000;	// will start the pulses
					}
				} else {
					SBusIndex = 0;
				}
			} else {
				if (SBusIndex && (TCnt - LastRcv) > 48000)					// 3 mS timeout		TODO 48000 depends on F_CPU
					SBusIndex = 0;
			}
		}
		
		readSBus();
		
		if ((micros() - Last16ChannelsStartTime) > 20000) {		// time for the first 4 pulses
			if (PulseStateMachine == ONE_TO_FOUR) {
				Last16ChannelsStartTime += 20000;
				setPulseTimes(PulseStateMachine);		// first 4 pulses
				PulseStateMachine = FIVE_TO_EIGHT;
				Last04ChannelsStartTime = micros();
			}
		}
		
		readSBus();
		
		if ((micros() - Last04ChannelsStartTime) > 3000) {		// time for the next 4 pulses
			switch (PulseStateMachine) {
				case FIVE_TO_EIGHT:
					setPulseTimes(PulseStateMachine);	// second 4 pulses
					if (EightOnly)
						PulseStateMachine = END_PULSES;
					else
						PulseStateMachine = NINE_TO_TWELVE;
					Last04ChannelsStartTime = micros();
				break;
				case NINE_TO_TWELVE:
					setPulseTimes(PulseStateMachine);	// third 4 pulses
					PulseStateMachine = THIRTEEN_TO_SIXTEEN;
					Last04ChannelsStartTime = micros();
				break;
				case THIRTEEN_TO_SIXTEEN:
					setPulseTimes(PulseStateMachine);	// fourth 4 pulses
					PulseStateMachine = END_PULSES;
					Last04ChannelsStartTime = micros();
				break;
			}
		}
		
		readSBus();

		if (PulseStateMachine == END_PULSES && PulseOutState == IDLE)
			PulseStateMachine = ONE_TO_FOUR;

		if ((millis() - LastSBusReceived) > 500)
			enterFailsafe();
		
		if (SBusHasBeenReceived == 0 && ( millis() - LastSBusReceived) > 100) {
			LastSBusReceived = millis();
			setSerialMode(!SerialMode);			// toggle mode for auto-detect
		}
		
		readSBus();
		
		checkFailsafePin();
	}
}
