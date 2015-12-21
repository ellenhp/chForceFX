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

#define min(A, B) ((A < B) ? A : B)
#define max(A, B) ((A > B) ? A : B)
#define clamp(val, lower, upper) min(upper, max(lower, val))

uint8_t PWM_on = 0;

void FFB_SetSpeedX(int8_t signedSpeed);
void FFB_SetSpeedY(int8_t signedSpeed);

void FFB_Init(void)
{
    DDRB |= (1 << PB5) | (1 << PB6) | (1 << PB4); // ADC X and Y, one direction enable for X
    DDRE |= (1 << PE6); // Other direction enable for X

    DDRD |= (1 << PD7); // One direction enable for Y
    DDRC |= (1 << PC6); // Other direction enable for Y

    PORTB &= ~(1 << PB4);
    PORTE &= ~(1 << PE6);

    PORTD &= ~(1 << PD7);
    PORTC &= ~(1 << PC6);

    wdt_set_interrupt_only(WDTO_60MS);
    FFB_Enable();
}

void FFB_Disable(void)
{
    FFB_SetSpeedX(0);
    FFB_SetSpeedY(0);

    TCCR1A = 0;
    TCCR1B = 0;

    PWM_on = 0;
}

void FFB_Enable(void)
{
    wdt_reset();

    TCCR1A = _BV(COM1A0) | _BV(COM1A1) | _BV(COM1B0) | _BV(COM1B1) | _BV(WGM10);
    TCCR1B = _BV(CS00);

    FFB_SetSpeedX(0);
    FFB_SetSpeedY(0);

    PWM_on = 1;
}

void FFB_SetSpeedY(int8_t signedSpeed)
{
    uint8_t direction = (signedSpeed >= 0) ? 1 : 0; //TODO: Does C99 guarantee the true condition here will result in 1?
    uint8_t notDirection = (signedSpeed >= 0) ? 0 : 1; //TODO: Does C99 guarantee !0 == 1?
    uint8_t pwmState;

    pwmState = (uint8_t)abs(signedSpeed);
    pwmState = 255 - (pwmState << 1);

    uint8_t portBState = PORTB & ~(1 << PB4);
    PORTB = portBState | (direction << PB4);

    uint8_t portEState = PORTE & ~(1 << PE6);
    PORTE = portEState | (notDirection << PE6);

    OCR1A = pwmState;
}

void FFB_SetSpeedX(int8_t signedSpeed)
{
    uint8_t direction = (signedSpeed >= 0) ? 1 : 0; //TODO: Does C99 guarantee the true condition here will result in 1?
    uint8_t notDirection = (signedSpeed >= 0) ? 0 : 1; //TODO: Does C99 guarantee !0 == 1?
    uint8_t pwmState;

    pwmState = (uint8_t)abs(signedSpeed);
    pwmState = 255 - (pwmState << 1);

    uint8_t portCState = PORTC & ~(1 << PC6);
    PORTC = portCState | (direction << PC6);

    uint8_t portDState = PORTD & ~(1 << PD7);
    PORTD = portDState | (notDirection << PD7);

    OCR1B = pwmState;
}

int8_t FFB_GetCenteringForce(int16_t axisVal, int16_t center)
{
    int16_t delta = axisVal - center;
    int8_t sign = (delta >= 0) ? 1 : -1;

    const int16_t deadzone = 10;

    delta = clamp(delta, -127, 127);

    int16_t centeringForce = (140 - abs(delta)*2/3) * sign ;// + sign * 20;

    return (int8_t) clamp(centeringForce, -127, 127);
}

void FFB_Update(uint8_t xAxis, uint8_t yAxis)
{
    wdt_reset();
    if (PWM_on)
    {
        FFB_SetSpeedX(FFB_GetCenteringForce(xAxis, 128));
        FFB_SetSpeedY(FFB_GetCenteringForce(yAxis, 128));
        // FFB_SetSpeedY(65);
    }
}

ISR(WDT_vect)
{
    FFB_Disable();
}
