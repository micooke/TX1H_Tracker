/*
Low power test for 8MHz boards
Note: This will work with other clock frequencies - but the delay routine will be wrong

Current draw during sleep
INT0  : WDT
301uA : 433.0uA  - without disable_peripherals()
0.6uA : 141.2uA  - using disable_peripherals()

*/

#include <Arduino.h>
#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down peripherals such as the ADC/TWI and Timers
#include <util/atomic.h>

// The ATtiny has the WDTCR register instead of the WDTCSR used by other AVRs
#ifndef WDTCSR
#define WDTCSR WDTCR
#endif

#define TX1H_SEND_SERIAL 0

#define TX1_SIGNAL 3 // PB3 - BLUE - TX1H SIGNAL
#define TX1_ON 4 // PB4 - GREEN - TX1H ON
#define MAX_PINS 6

#if (TX1H_SEND_SERIAL == 1)
#include <SoftwareUart.h>
SoftwareUart<> tx1h(-1, TX1_SIGNAL);
#endif

// Function declaration
void enable_ExternalInterrupt();
void disable_ExternalInterrupt();
void disable_peripherals();
void power_down();

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
void enable_watchdogInterrupt(uint8_t timerPrescaler = 6); // Default to 1s
void soft_delay_ms(uint16_t x);
void soft_delay_us(uint16_t x);

// ISR : Watchdog Interrupt
ISR(WDT_vect) {}
// ISR : External Interrupt
ISR(INT0_vect) {}

void setup()
{
	//To reduce power, setup all pins as inputs with no pullups
	// (saves about 1.3uA when used with watchdog timer in this example)
	for (uint8_t x = 0; x < MAX_PINS; ++x)
	{
		pinMode(x, INPUT);
		digitalWrite(x, (x == 2)); // set PB2 / INT0 high
	}

	// Set the LEDs as outputs
	pinMode(TX1_SIGNAL, OUTPUT);
	pinMode(TX1_ON, OUTPUT);

	digitalWrite(TX1_ON, HIGH);
	soft_delay_ms(1000);
	digitalWrite(TX1_ON, LOW);

	//Power down various bits of hardware to lower power usage
	disable_peripherals();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	//setup wakeup on external trigger
	enable_ExternalInterrupt();
	power_down(); // Go to sleep
	disable_ExternalInterrupt();

	//Setup wakeup on watchdog timer
	//Set watchdog to go off after 2sec
	enable_watchdogInterrupt(7);

#if (TX1H_SEND_SERIAL == 1)
	tx1h.begin(9600);
#endif
}

void loop()
{
	//Turn TX1H on, send your signal
	digitalWrite(TX1_ON, HIGH);
#if (TX1H_SEND_SERIAL == 1)
	tx1h.write(0xAA); // 10101010
#else
	//for (uint8_t i = 0; i < 5; ++i)
	//{
		digitalWrite(TX1_SIGNAL, HIGH);
		soft_delay_ms(50);
		digitalWrite(TX1_SIGNAL, LOW);
		soft_delay_ms(50);
		digitalWrite(TX1_SIGNAL, HIGH);
		soft_delay_ms(50);
		digitalWrite(TX1_SIGNAL, LOW);
	//	soft_delay_ms(50);
	//}
#endif
	digitalWrite(TX1_ON, LOW);

	power_down(); // Go to sleep
	// wake up here
}

void disable_peripherals()
{
	ADCSRA &= ~_BV(ADEN); //Disable ADC
	ACSR = _BV(ACD); //Disable the analog comparator

	DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-3 pins and disable digital input buffer on AIN0,1

#if defined(__AVR_ATtiny85__)
	power_timer0_disable(); //Needed for delay_ms
	power_timer1_disable();
	power_adc_disable();
	power_usi_disable();
#else
// Note: power_all_disable() works for all chips - it is expanded for ATtiny85 for clarity
	power_all_disable();
#endif
}

void power_down()
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		sleep_enable();
		sleep_bod_disable(); // Doesn't save any power. I assume it uses the fuze settings, which have BOD disabled
	}
	sleep_cpu();
	sleep_disable();
}

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=125ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
void enable_watchdogInterrupt(uint8_t timerPrescaler)
{
	// WDTCSR = [  WDIF |  WDIE |  WDP3 |  WDCE |  WDE  |  WDP2 |  WDP1 |  WDP0 ]

	uint8_t WDTCSR_ = (timerPrescaler & 0x07);
	WDTCSR_ |= (timerPrescaler > 7) ? _BV(WDP3) : 0x00; // Set WDP3 if prescalar > 7 (ie. 4.0s, 8.0s)
	WDTCSR_ |= _BV(WDIE); // Enable watchdog interrupt

	//This order of commands is important and cannot be combined (beyond what they are below)
	MCUSR &= ~_BV(WDRF); // Clear the watch dog reset

	// timed sequence
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		WDTCSR |= _BV(WDCE) | _BV(WDE); //Set WD_change enable, set WD enable
		WDTCSR = WDTCSR_; //Set new watchdog timeout value & enable
	}
}

void enable_ExternalInterrupt()
{
	// MCUCR = [  BODS |  PUD  |  SE   |  SM1  |  SM0  | BODSE | ISC01 | ISC00 ]
	//MCUCR |= _BV(ISC00); // ISC0[1,0] = [1,0] = rising edge detection
	// ISC0[1,0] = [0,1] = pin change detection
	MCUCR &= ~(_BV(ISC01) | _BV(ISC00)); // 0 : low level
	//MCUCR &= ~_BV(ISC01); MCUCR |= _BV(ISC00); // 1 : logical change
	//MCUCR |= _BV(ISC01); MCUCR &= ~_BV(ISC00); // 2 : falling edge
	//MCUCR |= _BV(ISC01) | _BV(ISC00); // 3 : rising edge


	// PCMSK = [   -   |   -   | PCINT5| PCINT4| PCINT3| PCINT2| PCINT1| PCINT0]
	//         [   -   |   -   |    RST|    PB4|    PB3|    PB2|    PB1|    PB0]
	PCMSK = _BV(PB2);// 0x00; // disable pin interrupts

	// GIMSK = [   -   |  INT0 |  PCIE |   -   |   -   |   -   |   -   |   -   ]
	GIMSK |= _BV(INT0); // Enable External Interrupt (INT0 / PB2), disable Pin Change Interrupt
}

void disable_ExternalInterrupt()
{
	PCMSK = 0x00;
	GIMSK = 0x00;
}

//This is a not-so-accurate delay routine
//Calling soft_delay_ms(100) will delay for about 100ms with a 8MHz clock
void soft_delay_ms(uint16_t x)
{
	for (; x > 0; x--)
	{
		soft_delay_us(1000);
	}
}

//This is a not-so-accurate delay routine
//Calling soft_delay_us(100) will delay for about 100us
//Assumes 8MHz clock
void soft_delay_us(uint16_t x)
{
	for (; x > 0; x--)
	{
		__asm__("nop\n\t");
		__asm__("nop\n\t");
		__asm__("nop\n\t");
		__asm__("nop\n\t");
		__asm__("nop\n\t");
		__asm__("nop\n\t");
		__asm__("nop\n\t");
	}
}

