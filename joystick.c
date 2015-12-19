#include "joystick.h"
#include "ffb.h"
#include "usb_hid.h"
#include <avr/io.h>

uint16_t read_adc(uint8_t channel){
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((channel >> 3) & 0x01) << MUX5);
	ADMUX = (1<<REFS0) | (channel & 0x7);
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC));

	uint8_t low, high;
	low  = ADCL;
	high = ADCH;

	return (high << 8) | low;                    //Returns the ADC value of the chosen channel
}

/** Configures the board hardware and chip peripherals for the joystick's functionality. */
void Joystick_Init(void)
{
    // Initialize..
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
    ADCSRA |= (1<<ADEN);                //Turn on ADC
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
	outReportData->X = (read_adc(7)>>2) - 128;
	outReportData->Y = (read_adc(6)>>2) - 128;

	outReportData->Button = 5;
	// outReportData->Hat = 1;

	return 1;
}
