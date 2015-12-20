#include "ffb.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define wdt_set_interrupt_only(value)   \
    __asm__ __volatile__ (  \
        "in __tmp_reg__,__SREG__" "\n\t"    \
        "cli" "\n\t"    \
        "wdr" "\n\t"    \
        "sts %0,%1" "\n\t"  \
        "out __SREG__,__tmp_reg__" "\n\t"   \
        "sts %0,%2" \
        : /* no outputs */  \
        : "M" (_SFR_MEM_ADDR(_WD_CONTROL_REG)), \
        "r" (_BV(_WD_CHANGE_BIT) | _BV(WDE)), \
			   "r" ((uint8_t) ((value & 0x08 ? _WD_PS3_MASK : 0x00) | \
					 _BV(WDIE) | (value & 0x07)) ) \
        : "r0"  \
    )

uint8_t PWM_on = 0;

uint8_t xAxisPWM_val = 255;
uint8_t yAxisPWM_val = 255;

void FFB_Init(void)
{
    DDRB |= (1 << PB5) | (1 << PB6);
    wdt_set_interrupt_only(WDTO_8S);
    wdt_reset();
    FFB_Enable();
}

void FFB_Disable(void)
{
    TCCR1A = 0;
    TCCR1B = 0;
    PWM_on = 0;
}

void FFB_Enable(void)
{
    TCCR1A = _BV(COM1A0) | _BV(COM1A1) | _BV(COM1B0) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(CS00);

    OCR1A = 127; //This keeps outputs in a logical LOW state. (0% duty cycle)
    OCR1B = 127;

    PWM_on = 1;
}

void FFB_Update(uint8_t xAxis, uint8_t yAxis)
{
    wdt_reset();
    if (PWM_on)
    {
        //Update stuff.
    }
}

ISR(WDT_vect)
{
    FFB_Disable();
}
