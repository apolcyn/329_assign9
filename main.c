#include <msp430.h>

#define VOLUME_UP 0x20DF40BF
#define VOLUME_DOWN 0x20DFC03F
#define CHANNEL_UP 0x20df00ff
#define CHANNEL_DOWN 0x20DF807F

#define ms_20        655   // 20.0 ms
#define ms_0point75  25    // 0.75 ms
#define ms_1point50  49    // 1.50 ms
#define ms_2point25  73    // 2.25 ms

int duty_cycles[] = {ms_0point75, ms_1point50, ms_2point25};

int done = 1;
long buf = 0;
int index = 0;
long timeCounter = 0;

/* Shifts the servo to some angle controlled by the duty_cycle parameter */
void shift_servo(int duty_cycle_len) {
	  P1DIR |= BIT2;                            // P1.2 output

	  CCTL0 &= ~CCIE;                           // Disable clock interrupts.
                                                // Not reading IR sensor right now.
	  // Using P1.2 for hardware set/reset as when TAR hits CCRO and CCR1 values.
	  // This allows P1.2 to toggle outside of software control.  Note that Timer A
	  // is running compare mode but interrupts are not being utilized.
	  // Note that this feature of toggling port bits outside of software control
	  // may be utilized while cpu is in low power mode.
	  P1SEL |= BIT2;                            // P1.2 TA1/2 options

	  // Setting CCR0 for a 20 ms period with CCTL1 determining pulse width (duty
	  // cycle) as determined by OUTMOD_7.
	  CCR0 = ms_20 - 1;                         // PWM Period of 20 ms
	  CCTL1 = OUTMOD_7;                         // CCR1 reset/set
	  TACTL = TASSEL_1 + MC_1;                  // ACLK, up mode

	  // Sample code to move the server about its range of movement

	  CCR1 = duty_cycle_len;                    // CCR1 PWM duty cycle
	  _delay_cycles(1000000);

	  TACTL = TASSEL_2 + MC_1;                  // Set clock back up for IR sensor reading.
	  TACCR0 = 100;
	  CCTL0 |= CCIE;
	  P1SEL &= ~BIT2;                           // Turn off duty cycle output
}

/*
 * main.c
 */
int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	          // Stop watchdog timer

    if (CALBC1_16MHZ==0xFF)	              // If calibration constant erased
    {
        while(1);                         // do not load, trap CPU!!
    }
    DCOCTL = 0;                           // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                // Set range
    DCOCTL = CALDCO_1MHZ;

    CCTL0 |= CCIE;
    TACTL = TASSEL_2 + MC_1;
    TACCR0 = 100;

    P1DIR |= BIT6;                        // UsingP1.6 for LED.

    P1IES |= BIT1;                        // Set P1.1 to trigger on a falling edge.
    P1IES &= ~BIT0;                       // Set P1.0 to trigger on a rising edge
    P1IE |= BIT0 + BIT1;

    __delay_cycles(250000);
    P1IFG = 0;                            // Clear out unintended GPIO triggerings.

    __enable_interrupt();

    while(1) {
    	buf = 0;                          // Reset buffer.
        while(index < 32)                 // Sit here until we decode a 32-bit instruction
        	;                             // from TV remote.
        index = 0;                        // Reset index, and set flag to indicate
        done = 1;                         // that the next faling edge is not a data bit.

        switch(buf) {                     // Perform some action based on the code
        case CHANNEL_UP:                  // sent by the TV remote.
        	shift_servo(duty_cycles[0]);
        	break;

        case CHANNEL_DOWN:
        	shift_servo(duty_cycles[1]);
        	break;

        case VOLUME_UP:
        	shift_servo(duty_cycles[2]);
        	break;

        case VOLUME_DOWN:
        	P1OUT ^= BIT6;
        	break;
        default:                           // Error condition. Read an invalid code.
        	index = 0;
        	break;
        }
    }

	return 0;
}

/* Increments timeCounter, which counts up with a 100us period. Used to decode IR. */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void something(void) {
	timeCounter++;
}

/* Reads the next data bit from IR sensor and stores it in a buffer.
 * Using NEC protocol, so decodes '1's or '0's based on width of high signal.
 */
#pragma vector=PORT1_VECTOR
__interrupt void button(void) {
	if(P1IFG & BIT0 && P1IFG & BIT1) {    // Check error condition.
		P1OUT ^= BIT6;                    // Shouldn't have interrupts for both high to low
		__delay_cycles(250000);           // and low to high transitions.
		P1OUT ^= BIT6;                    // Toggle LED to indicate error.
		__delay_cycles(250000);
		P1OUT ^= BIT6;
	}
	else if(P1IFG & BIT0) {               // Rising edge from IR receiver digital output.
		if(done) {                        // Indicate that subsequent edges are of interest.
			done = 0;
		}
		timeCounter = 0;                  // Reset time counter to zero on rising edge.
	}
	else if(P1IFG & BIT1 && !done) {      // Falling edge from IR receiver digital output.
		long diff = timeCounter;          // See how many 100us periods have gone by since
                                          // the previous rising edge.
		if(diff < 8) {                    // This high signal was less than 800us long,                          // '0'
			index++;                      // which inidicates a '0' with NEC protocol.
			buf <<= 1;                    // Add in a '0' bit to the buffer.
		}
		else if(diff < 20) {              // This high signal is between 800us and 2ms,
			index++;                      // which inidicates a '1' bt by NEC protocol.
			buf <<= 1;                    // Add in a '1' bit to the buffer.
			buf |= 1;
		}
		else if(diff < 60) {              // This high signal is between 2ms and 6ms,
			if(buf | index != 0) {        // which inidicates that this was the data header.
				P1OUT |= BIT6;            // If buffer and index and not both set to zero,
			}                             // then this is an error condition, as these should
		}                                 // be reset before data capture.
	}
	__delay_cycles(100);                  // Delay a short bit just to prevent
	                                      // possible unintended edge triggerings.
	P1IFG = 0;                            // Clear Port 1 GPIO interrup flags
}
