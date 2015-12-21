#include "joystick.h"
#include "ffb.h"
#include "usb_hid.h"
#include <avr/io.h>

static uint8_t xAxisValues[128];
static uint8_t yAxisValues[128];

static uint8_t xPtr = 0;
static uint8_t yPtr = 0;

static uint8_t adcChannel = 6;

/** Configures the board hardware and chip peripherals for the joystick's functionality. */
void Joystick_Init(void)
{
    // Initialize..
	ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0)); //16Mhz/128 = 125Khz the ADC reference clock
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcChannel >> 3) & 0x01) << MUX5);
	ADMUX = (1 << REFS0) | (adcChannel & 0x7);
    ADCSRA |= (1 << ADEN); // Enable on ADC
	ADCSRA |= (1 << ADIE); // Enable ADC Interrupt
	ADCSRA |= (1 << ADSC); // Run ADC
}

/** Configures the board hardware and chip peripherals for the joystick's functionality. */
int Joystick_Connect(void)
{
	return 1;
}


int Joystick_CreateInputReport(uint8_t inReportId, USB_JoystickReport_Data_t* const outReportData)
{
	// Convert the raw input data to USB report
	outReportData->reportId = 1;

	int16_t xTotal = 0;
	int16_t yTotal = 0;
	for (int i = 0; i < sizeof(xAxisValues); i++)
	{
		xTotal += xAxisValues[i];
		yTotal += yAxisValues[i];
	}
	outReportData->X = (xTotal / sizeof(xAxisValues)) - 128;
	outReportData->Y = (yTotal / sizeof(yAxisValues)) - 128;

	return 1;
}

ISR(ADC_vect)
{
	uint8_t low, high;
	low  = ADCL;
	high = ADCH;
	if (adcChannel == 6)
	{
		adcChannel = 7;
		xAxisValues[xPtr] = (high << 6) | (low >> 2);
		xPtr = (xPtr + 1) % sizeof(xAxisValues);
	}
	else
	{
		adcChannel = 6;
		yAxisValues[yPtr] = (high << 6) | (low >> 2);
		yPtr = (yPtr + 1) % sizeof(yAxisValues);
	}

	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcChannel >> 3) & 0x01) << MUX5);
	ADMUX = (1 << REFS0) | (adcChannel & 0x7);
	ADCSRA |= (1 << ADSC);
}
