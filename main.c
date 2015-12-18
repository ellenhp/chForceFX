#include "main.h"
#include "joystick.h"
#include "usb_hid.h"
#include "ffb.h"

int main(void)
{
	SetupHardware();

	LEDs_SetAllLEDs(LEDS_NO_LEDS);
	sei();

	while (1)
	{
		HID_Task();

		CDC1_Task();

		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	LEDs_Init();

	USB_Init();
}

/** Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
 *  starts the library USB task to begin the enumeration and USB management process.
 */
void EVENT_USB_Device_Connect(void)
{
	/* Indicate USB enumerating */
	LEDs_SetAllLEDs(LEDS_ALL_LEDS);
}

/** Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
 *  the status LEDs and stops the USB management and joystick reporting tasks.
 */
void EVENT_USB_Device_Disconnect(void)
{
	/* Indicate USB not ready */
	LEDs_SetAllLEDs(LEDS_NO_LEDS);
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
 *  of the USB device after enumeration - the device endpoints are configured and the joystick reporting task started.
 */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	/* Setup HID Report Endpoint */
	ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPNUM, EP_TYPE_INTERRUPT,
	                                            JOYSTICK_EPSIZE, 1);

	ConfigSuccess &= Endpoint_ConfigureEndpoint(FFB_EPNUM, EP_TYPE_INTERRUPT,
	                                            FFB_EPSIZE, 1);

	/* Indicate endpoint configuration success or failure */
	LEDs_SetAllLEDs(ConfigSuccess ? LEDS_NO_LEDS : LEDS_ALL_LEDS);
}


volatile bool sPIDStatusPending = false;
volatile USB_FFBReport_PIDStatus_Input_Data_t sPIDStatus;


/** Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
 *  the device from the USB host before passing along unhandled control requests to the library for processing
 *  internally.
 */
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */

	/*
	USB_ControlRequest:
		uint8_t 	bmRequestType
		uint8_t 	bRequest
		uint16_t 	wValue
		uint16_t 	wIndex
		uint16_t 	wLength
	*/

	switch (USB_ControlRequest.bRequest)
	{
		// Joystick stuff

		case HID_REQ_GetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				LEDs_SetAllLEDs(LEDS_ALL_LEDS);

				/*if (USB_ControlRequest.wValue == 0x0306)
					{	// Feature 2: PID Block Load Feature Report
					_delay_us(500);	// Windows needs this delay to register the below feature report correctly
					USB_FFBReport_PIDBlockLoad_Feature_Data_t featureData;
					FfbOnPIDBlockLoad(&featureData);
					Endpoint_ClearSETUP();
					// Write the report data to the control endpoint
					Endpoint_Write_Control_Stream_LE(&featureData, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
					Endpoint_ClearOUT();
					}
				else */
				if (USB_ControlRequest.wValue == 0x0307)
				{	// Feature 3: PID Pool Feature Report
					USB_FFBReport_PIDPool_Feature_Data_t featureData;
					//FfbOnPIDPool(&featureData);

					Endpoint_ClearSETUP();

					// Write the report data to the control endpoint
					Endpoint_Write_Control_Stream_LE(&featureData, sizeof(USB_FFBReport_PIDPool_Feature_Data_t));
					Endpoint_ClearOUT();
				}
				else
				{
					USB_JoystickReport_Data_t JoystickReportData;

					/* Create the next HID report to send to the host */
					Joystick_CreateInputReport(USB_ControlRequest.wValue & 0xFF, &JoystickReportData);

					Endpoint_ClearSETUP();

					/* Write the report data to the control endpoint */
					Endpoint_Write_Control_Stream_LE(&JoystickReportData, sizeof(USB_JoystickReport_Data_t));
					Endpoint_ClearOUT();
				}
			}

			break;
		case HID_REQ_SetReport:
			if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
			{
				Endpoint_ClearSETUP();

				uint8_t data[10];	// This is enough room for all reports
				uint16_t len = 0;	// again, enough for all

				len = USB_ControlRequest.wLength;

				// Read in the report data from host

				// Read the report data from the control endpoint
				Endpoint_Read_Control_Stream_LE(&data, len);
				Endpoint_ClearStatusStage();

				// Process the incoming report
				if (USB_ControlRequest.wValue == 0x0305)
				{	// Feature 1
					_delay_us(500);	// Windows does not like to be answered too quickly

					USB_FFBReport_PIDBlockLoad_Feature_Data_t pidBlockLoadData;
					// FfbOnCreateNewEffect((USB_FFBReport_CreateNewEffect_Feature_Data_t*) data, &pidBlockLoadData);

					Endpoint_ClearSETUP();

					// Write the report data to the control endpoint
					Endpoint_Write_Control_Stream_LE(&pidBlockLoadData, sizeof(USB_FFBReport_PIDBlockLoad_Feature_Data_t));
					Endpoint_ClearOUT();
				}
				else if (USB_ControlRequest.wValue == 0x0306)
				{
				}
				else if (USB_ControlRequest.wValue == 0x0307)
				{
				}
            }
		break;

	}

}

/** Function to manage HID report generation and transmission to the host. */
void HID_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* Select the Joystick Report Endpoint */
	Endpoint_SelectEndpoint(JOYSTICK_EPNUM);

	/* Check to see if the host is ready for another packet */
	if (Endpoint_IsINReady())
	{
		USB_JoystickReport_Data_t JoystickReportData;

		/* Create the next HID report to send to the host */
		// Joystick_CreateInputReport(INPUT_REPORTID_ALL, &JoystickReportData);

		/* Write Joystick Report Data */
		Endpoint_Write_Stream_LE(&JoystickReportData, sizeof(USB_JoystickReport_Data_t), NULL);

		/* Finalize the stream transfer to send the last packet */
		Endpoint_ClearIN();
	}

	// Receive FFB data
	Endpoint_SelectEndpoint(FFB_EPNUM);

	if (Endpoint_IsOUTReceived())
	{
		LEDs_SetAllLEDs(LEDS_ALL_LEDS);

		uint8_t out_ffbdata[64];	// enough for any single OUT-report
		uint8_t total_bytes_read = 0;

		while (Endpoint_BytesInEndpoint() && total_bytes_read < 64)
		{
			uint16_t out_wait_report_bytes = 0;

			// Read the reportID from the package to determine amount of data to expect next
			while (Endpoint_Read_Stream_LE(&out_ffbdata, 1, NULL)
					 == ENDPOINT_RWSTREAM_IncompleteTransfer)
			{	// busy loop until the first byte is read out
			}

			total_bytes_read += 1;

			while (Endpoint_Read_Stream_LE(&out_ffbdata[1], out_wait_report_bytes, NULL)
					== ENDPOINT_RWSTREAM_IncompleteTransfer)
			{	// busy loop until the rest of the report data is read out
			}

//			LogData("Read OUT data:", out_ffbdata[0], &out_ffbdata[1], out_wait_report_bytes);
			total_bytes_read += out_wait_report_bytes;

			// FfbOnUsbData(out_ffbdata, out_wait_report_bytes + 1);
			_delay_ms(1);
		}

		// Clear the endpoint ready for new packet
		Endpoint_ClearOUT();
	}
}


#if !defined ENABLE_JOYSTICK_SERIAL
void CDC1_Task(void)  {}
#else

void ProcessDataFromCOMSerial(char data);

void CDC1_Task(void)
{
	/* Device must be connected and configured for the task to run */
	if (USB_DeviceState != DEVICE_STATE_Configured)
		return;

	/* Select the Serial Rx Endpoint */
	Endpoint_SelectEndpoint(CDC1_RX_EPNUM);

	char data[128];
	uint16_t len = 0;

	/* Throw away any received data from the host */
	if (Endpoint_IsOUTReceived())
	{
		LEDs_SetAllLEDs(LEDS_ALL_LEDS);

		while (Endpoint_BytesInEndpoint() && len < 128)
		{
			// Read the reportID from the package to determine amount of data to expect next
			while (Endpoint_Read_Stream_LE(&data[len++], 1, NULL)
					 == ENDPOINT_RWSTREAM_IncompleteTransfer)
			{	// busy loop until the first byte is read out
			}
		}

		Endpoint_ClearOUT();

		for (uint16_t i = 0; i < len; i++)
		{
			ProcessDataFromCOMSerial(data[i]);
		}

		LEDs_SetAllLEDs(LEDS_NO_LEDS);
	}
}

uint8_t ParseHexNibble(char data)
{
	if (data >= '0' && data <= '9')
		return data - '0';
	else if (data >= 'a' && data <= 'f')
		return (data - 'a') + 10;
	else if (data >= 'A' && data <= 'F')
		return (data- 'A') + 10;
	else
		return 0xFF; // not a hex nibble
}


void DoCommandListEffects(void);
void DoCommandSetEffectType(char effectType, char value);
void DoCommandSetEffectAtIndex(uint8_t effectIndex, char value);
void DoCommandSendMidi(uint8_t *data, uint16_t len);
void DoCommandSimulateUsbReceive(uint8_t *data, uint16_t len);

void ProcessCommandDataFromCOMSerial(char command, char data);

volatile static uint8_t gOngoingSerialCommandDataLen = 0; // expected length of actual command data
volatile static char gOngoingSerialCommand = '\0';

void ProcessDataFromCOMSerial(char data)
{
	volatile static uint8_t gOngoingSerialCommandParameterPos = 0; // how many parameter nibbles have been read
	volatile static uint8_t gDataByte = 0; // currently parsed data byte cache

	// Check for start of a new command
	if (gOngoingSerialCommand == 0)
	{
		if (data <= 32 || data > 'z') return; // some sanity checking - skip possible garbage or formatting

		// Commands with no parameters can be handled directly here
		if (data == 'l')
		{
			//DoCommandListEffects();
			return;
		}

		// The command has parameter data - need to parse and collect them nibble by nibble
		gOngoingSerialCommand = data;
		gOngoingSerialCommandParameterPos = 0;
		gOngoingSerialCommandDataLen = 0;
		return;
	}

	// Check for data length parameter (2 hexadecimal nibbles)
	if (gOngoingSerialCommandParameterPos == 0)
	{
		uint8_t value = ParseHexNibble(data);
		if (value == 0xFF)
			return;
		gOngoingSerialCommandParameterPos = 1;
		gOngoingSerialCommandDataLen = value << 4;
		return;
	}

	if (gOngoingSerialCommandParameterPos == 1)
	{
		uint8_t value = ParseHexNibble(data);
		if (value == 0xFF)
			return;
		gOngoingSerialCommandParameterPos = 2;
		gOngoingSerialCommandDataLen += value;

		return;
	}

	// Parse actual data
	uint8_t value = ParseHexNibble(data);
	if (value == 0xFF)
		return;
	gOngoingSerialCommandParameterPos++;
	if (gOngoingSerialCommandParameterPos % 2)
	{
		gDataByte = value << 4;
	}
	else
	{
		gDataByte = gDataByte + value;
		ProcessCommandDataFromCOMSerial(gOngoingSerialCommand, gDataByte);
	}
}

void CompletedCommandDataFromCOMSerial(char command, char *data, uint16_t len);

void ProcessCommandDataFromCOMSerial(char command, char data)
{
	// Reserve a buffer for sending raw-data from COM serial to USB/MIDI handling
#define SERIAL_COMMAND_BUFFER_SIZE 40
	static char SERIAL_COMMAND_BUFFER[SERIAL_COMMAND_BUFFER_SIZE];
	volatile static uint8_t gOngoingSerialCommandDataPos = 0; // writer offset of command data in SERIAL_COMMAND_BUFFER

	SERIAL_COMMAND_BUFFER[gOngoingSerialCommandDataPos] = data;
	gOngoingSerialCommandDataPos++;

	if (gOngoingSerialCommandDataPos >= gOngoingSerialCommandDataLen)
	{ // all data received - command completely received
		CompletedCommandDataFromCOMSerial(command, SERIAL_COMMAND_BUFFER, gOngoingSerialCommandDataLen);
		gOngoingSerialCommand = 0; // wait for the next command
		gOngoingSerialCommandDataPos = 0;
	}
}

void CompletedCommandDataFromCOMSerial(char command, char *data, uint16_t len)
{
	if (command == 'm')
		DoCommandSendMidi((uint8_t*) data, len);
	else if (command == 'u')
		DoCommandSimulateUsbReceive((uint8_t*) data, len);
	else if (command == 't') // disable effect type
		DoCommandSetEffectType(data[0], 0);
	else if (command == 'T') // enable effect type
		DoCommandSetEffectType(data[0], 1);
	else if (command == 'e') // disable effect at index
		DoCommandSetEffectAtIndex(data[0], 0);
	else if (command == 'E') // enable effect at index
		DoCommandSetEffectAtIndex(data[0], 1);
}

void DoCommandSetEffectType(char effectType, char value)
{
	// if (effectType == 8)
	// 	FfbEnableSprings(value);
	// else if (effectType == 1)
	// 	FfbEnableConstants(value);
	// else if (effectType == 5)
	// 	FfbEnableTriangles(value);
	// else if (effectType == 2)
	// 	FfbEnableSines(value);
}

void DoCommandSetEffectAtIndex(uint8_t effectIndex, char value)
{
	// FfbEnableEffectId(effectIndex, value);
}

void DoCommandSendMidi(uint8_t *data, uint16_t len)
{
	// FfbSendData(data, len);
}

void DoCommandSimulateUsbReceive(uint8_t *data, uint16_t len)
{
	// FfbOnUsbData(data, len);
}

#endif //ENABLE_JOYSTICK_SERIAL
