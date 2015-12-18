#include "joystick.h"
#include "ffb.h"
#include "usb_hid.h"

const uint8_t INPUT_REPORTID_ALL = 0xFF;

/** Configures the board hardware and chip peripherals for the joystick's functionality. */
void Joystick_Init(void)
{
    // Initialize..

	// ADC for extra controls
	DDRF = 0; // all inputs
//	PORTF |= 0xff; // all pullups enabled

	// Init and enable ADC
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX |= (1 << REFS0);		// ADC reference := Vcc
//	ADCSRA |= (1 << ADFR); 		// ADATE
//	ADCSRA |= (1 << ADATE); 	// free/continous mode
	ADMUX |= (1 << ADLAR); 		// 8-bit resolution to HIGH
	ADCSRA |= (1 << ADEN); 		// Enable ADC
	ADCSRA |= (1 << ADIE);  	// Enable ADC Interrupt

	ADCSRA |= (1 << ADSC); 		// Go ADC
	}

/** Configures the board hardware and chip peripherals for the joystick's functionality. */
int Joystick_Connect(void)
{
	return 1;
}

/** Fills the given HID report data structure with the next HID report to send to the host.
 *
 *  \param[out] outReportData  Pointer to a HID report data structure to be filled
 *
 *  \return Boolean true if the new report differs from the last report, false otherwise
 */
typedef struct
{
	int16_t 	position;
	uint16_t 	buttons;
	uint8_t 	others;
	uint16_t 	scaler;
} JoystickData;

typedef struct
{
	uint8_t sampledChannel;
	uint8_t	trim1, trim2, pedal1, pedal2;
} AddedControls_ADC_t;

static volatile AddedControls_ADC_t added_controls_adc;

static volatile JoystickData prev_joystick_data;
static volatile uint8_t ADC_is_ready = 0;



int Joystick_CreateInputReport(uint8_t inReportId, USB_JoystickReport_Data_t* const outReportData)
{
	// ???? This could be done more directly by modifying the 3DPVert code
	// ???? that generates its own USB report to the abovementioned format.
	// Here we read the additional analog controls in
	// rotation for each AD-channel at a time.
	//
	// Here,
	//	ADC0 - Trim 1
	//	ADC1 - Trim 2
	//	ADC4 - Left Pedal
	//	ADC5 - Right Pedal

	if (ADC_is_ready)
	{
		ADC_is_ready = 0;

		switch (added_controls_adc.sampledChannel)
		{
			case 0:
				added_controls_adc.trim1 = ADCH;
				added_controls_adc.sampledChannel = 1;
				break;
			case 1:
				added_controls_adc.trim2 = ADCH;
				added_controls_adc.sampledChannel = 4;
				break;
			case 4:
				added_controls_adc.pedal1 = ADCH;
				added_controls_adc.sampledChannel = 5;
				break;
			case 5:
				added_controls_adc.pedal2 = ADCH;
				added_controls_adc.sampledChannel = 0;
				break;
			default:
				added_controls_adc.sampledChannel = 0;	// just in case
				break;
		}

		// Start AD-conversion for the next channel in the rotation
		ADMUX &= 0b11111000;
		ADMUX |= (0b111 & added_controls_adc.sampledChannel);
		ADCSRA|=(1<<ADSC);
	}

/*
5 wwwwwwww
4 aaaaaaww
3 BAbbbbbb
2 1FLZYXRC
1 p-------
0 --------
*/

	// Convert the raw input data to USB report

	outReportData->reportId = 1;	// Input report ID 1

	// Convert data from Sidewinder Force Feedback Pro
	outReportData->X = 100;
	outReportData->Y = 101;
	outReportData->Button = 5;
	outReportData->Hat = 1;

	return 1;
}


// AD-conversion-completed-interrupt handler.
// We only set a "ADC ready"-flag here so that
// the actual data can be read later when there
// is more time. Doing any heavier stuff here
// seems to cause problems reading data from FFP joystick.
ISR(ADC_vect)
{
	ADC_is_ready = 1;
}
