#include "joystick.h"
#include "ffb.h"
#include "usb_hid.h"
#include <avr/io.h>

#define min(A, B) ((A < B) ? A : B)
#define max(A, B) ((A > B) ? A : B)
#define clamp(val, lower, upper) min(upper, max(lower, val))

static int8_t axisValues[2] = {0, 0};
static uint16_t axisAccumulators[2] = {0, 0};

static uint8_t accumulatedValueCount = 0;

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
	outReportData->X = axisValues[0];
	outReportData->Y = axisValues[1];

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
		axisAccumulators[0] += (high << 6) | (low >> 2);
	}
	else
	{
		adcChannel = 6;
		axisAccumulators[1] += (high << 6) | (low >> 2);
		if (accumulatedValueCount < 8)
		{
			accumulatedValueCount++;
		}
		else
		{
			int16_t positiveValue = axisAccumulators[0] >> 3;
			axisValues[0] = (int8_t)clamp(positiveValue - 128, -127, 127);

			positiveValue = axisAccumulators[1] >> 3;
			axisValues[1] = (int8_t)clamp(positiveValue - 128, -127, 127);

			axisAccumulators[0] = 0;
			axisAccumulators[1] = 0;
			accumulatedValueCount = 0;

			//TODO: This is probably a hack. Get another timer working so we can put it on another ISR.
			FFB_Update(axisValues[0], axisValues[1]);
		}

	}

	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcChannel >> 3) & 0x01) << MUX5);
	ADMUX = (1 << REFS0) | (adcChannel & 0x7);
	ADCSRA |= (1 << ADSC);
}
