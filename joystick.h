#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include "main.h"

extern const uint8_t INPUT_REPORTID_ALL;

// Data structures for input reports from the joystick positions

typedef struct
{
	// Joystick Input Report
	uint8_t	reportId;	// =1
	uint8_t  X;
	uint8_t  Y;
	uint16_t Button;
	// uint8_t Hat;
} USB_JoystickReport_Data_t;

// Functions that form the inferface from the generic parts of the code
// to joystick model specific parts.


// Gets called at very beginning to allow joystick model specific
// initializations of the hardware and software to occur.
void Joystick_Init(void);

// Gets called to check connection to the joystick at startup
// or after disconnect.
// If initialization succeeds, the function should return true.
// If it fails, return false and this functions gets called again
// until further progress is made (e.g. waiting for the actual
// joystick to be connected).
int Joystick_Connect(void);

// Gets called when input report of joysticks position, buttons etc. are
// requested. Data written to <outReportData> is sent to host if the function
// returns true. If false is returned, nothing is sent.
// If <inReportId> has value INPUT_REPORTID_ALL, all input report IDs should
// generated.
int Joystick_CreateInputReport(uint8_t inReportId, USB_JoystickReport_Data_t* const outReportData);

#endif
