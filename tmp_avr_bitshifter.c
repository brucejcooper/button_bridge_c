

#define RELAY PB1
#define DIN PB2
#define DOUT PB0



PORTB = (1 << DOUT); // Default to high level for OUT and low for RELAY
PUEB = (1 << DIN); // Enable pullup on DIN;
DDRB = (1 << RELAY) | (1 << DOUT); // Set Pin DOUT and RELAY to Output, all others inputs.


ACSR = 1 << ACSR_ACD; // Disable ADC
PRR = 1 << PRR_PRADC; // Power off ADC
SMCR = SMCR_SE | SMCR_POWER_DOWN; // Enable Sleep, and set mode to DEEP 
    
// Set Compare register to cause overflow after our timeout period. This is started at the start of each bit.
// So the actual idle period will be this minus one period. 
ICR0 = 100; // raise TOV0 flag after one full period.

// set output capture to set output to high when timer == OCR0A value (default to higher then interrupt value)


// Set DIN to generate INT0 interrupt when it goes low


uint8_t bit = 0;
uint8_t relay = 0;

for (;;) {
    // Wait for a falling edge (No timeout) of first bit
    EIFR = (1<<INTF0); // Clear INT0 Flag
    EICRA = ICS0_LOW;
    sleep(); // Deep Sleep.  Can only be woken by INT0
    EICRA = 0; // Turn off INT0 again. 

    // setup timer to manage just timeout right now.  Once we have read one bit we will add in output compare to return DOUT high
    TCNT0 = 3;
    TCCR0A = WGM_MODE_CTC_ICR0_LOWBITS;
    TCCR0B = WGM_MODE_CTC_ICR0_HIGHBITS | TIMER_CLK_DIV_1; // Starts the timer;
    SMCR = SMCR_SE | SMCR_IDLE; // set sleep mode to IDLE mode (CPU halted but timers still running)

    // Wait 50us
    sleep_us(50-5);
    bit = PORTB & (1 << DIN) ? 1 : 0;   // Read DIN


    for (;;) {     
        // Ensure that DIN goes high again.  If timeout occurs while waiting, this is a reset condition
        EIFR = (1<<INTF0); // Clear INT0 Flag
        EICRA = ICS0_HIGH;
        sleep(); // Shallow sleep, can be woken up by timer or by INT0
        if (TIFR0 & (1 << TOV0)) {
            // Bus has been held low for a full period.  Consider this a full reset of the device.
            // But first, propagate the reset to downstream.  This will ripple out.
            PORTB &= ~(1 << DOUT); 
            sleep_us(110);
            PORTB |= (1 << DOUT); 
            TIFR0 = (1 << TOV0);  // Reset Timer Flag.
            reset();
        }

        // Wait for Negative Edge (or timeout)
        EIFR = (1<<INTF0); // Clear INT0 Flag
        EICRA = ICS0_LOW;
        sleep(); // Shallow sleep, can be woken up by timer or by INT0
        EICRA = 0;
        // If timeout occurred. 
        if (TIFR0 & TOV0) {
            // Bus now idle. 
            break;
        }
        PORTB &= ~(1 << DOUT) // write low (start bit). Timer will raise it again after OCR0A uSec
        TCNT0 = 0; // Reset the timer.
        TCCR0A = SET_OC0A_ON_COMPARE_MATCH | WGM_MODE_CTC_ICR0_LOWBITS; // set DOUT high when TCLK0 == ORC0A
        // set output capture timer to 250us if bit = 1, or 750 if bit = 0
        OCR0A = bit ? 25 : 75;
        // wait 50 usec
        sleep_us(50-6-6);

        bit = PORTB & (1 << DIN) ? 1 : 0;   // Read DIN
        sleep_us(30);  // Wait enough time that the timer to raise PB0 again 
    }

    // Bus is idle.
    TCCR0B = WGM_MODE_CTC_ICR0_HIGHBITS | TIMER_CLK_STOP; // Stops the timer.
    TIFR0 = (1 << TOV0); // Clear Timer overflow flag

    PORTB = bit ? PORTB | (1 << RELAY) : PORTB & ~(1 << RELAY); // Set or clear relay bit to our current value
}