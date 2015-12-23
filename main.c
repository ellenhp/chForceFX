#include "main.h"
#include "joystick.h"
#include "usb_hid.h"
#include "ffb.h"

uint16_t bootKey = 0x7777;
uint16_t *const bootKeyPtr = (uint16_t *)0x0800;

int main(void)
{
    SetupHardware();

    sei();

    for (;;)
    {
        HID_Task();

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

    USB_Init();
    Joystick_Init();
	FFB_Init();
}

/** Event handler for the USB_ConfigurationChanged event. This is fired when the host set the current configuration
*  of the USB device after enumeration - the device endpoints are configured and the joystick reporting task started.
*/
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    /* Setup HID Report Endpoint */
    ConfigSuccess &= Endpoint_ConfigureEndpoint(JOYSTICK_EPNUM | ENDPOINT_DIR_IN, EP_TYPE_INTERRUPT,
                        JOYSTICK_EPSIZE, 1);

    ConfigSuccess &= Endpoint_ConfigureEndpoint(FFB_EPNUM | ENDPOINT_DIR_OUT, EP_TYPE_INTERRUPT,
                        FFB_EPSIZE, 1);
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
                    Endpoint_ClearIN();
                }
                else
                {
                    USB_JoystickReport_Data_t joystickReportData;

                    /* Create the next HID report to send to the host */
                    Joystick_CreateInputReport(USB_ControlRequest.wValue & 0xFF, &joystickReportData);

                    Endpoint_ClearSETUP();

                    /* Write the report data to the control endpoint */
                    Endpoint_Write_Control_Stream_LE(&joystickReportData, sizeof(USB_JoystickReport_Data_t));
                    Endpoint_ClearIN();
                }
            }

            break;
        case HID_REQ_SetReport:
            if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
            {
                Endpoint_ClearSETUP();

                uint8_t data[10];	// This is enough room for all reports
                uint16_t len = 10;	// again, enough for all

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
                else if (USB_ControlRequest.wValue == 0x0309)
                {	// Feature 1
                    uint8_t report[2] = {0x09, 0x01};

                    Endpoint_ClearSETUP();

                    // Acknowledge the request.
                    Endpoint_Write_Control_Stream_LE(&report, sizeof(report));
                    Endpoint_ClearOUT();

                    cli();
                    *bootKeyPtr = bootKey;  // There is no way this memory will ever be accessed again until reboot.
					wdt_reset();
					wdt_disable();
                    wdt_enable(WDTO_250MS); // Rationale: Interrupts are off, and an infinite loop follows.
                    while (1);
                }
                else if (USB_ControlRequest.wValue == 0x030F)
                {
                    int8_t xCenterIn, yCenterIn;
                    memcpy(&xCenterIn, &data[0], 1);
                    memcpy(&yCenterIn, &data[1], 1);

                    int16_t pIn = data[2];
                    int16_t iIn = data[3];
                    int16_t dIn = data[4];

                    FFB_SetPID(pIn, iIn, dIn);
                    FFB_SetCenter(xCenterIn, yCenterIn);

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

    if (Endpoint_IsINReady())
    {
        USB_JoystickReport_Data_t joystickReportData;
        Joystick_CreateInputReport(1, &joystickReportData);

        /* Write Joystick Report Data */
        Endpoint_Write_Stream_LE(&joystickReportData, sizeof(USB_JoystickReport_Data_t), NULL);

        /* Finalize the stream transfer to send the last packet */
        Endpoint_ClearIN();
    }
    // Receive FFB data
    Endpoint_SelectEndpoint(FFB_EPNUM);

    if (Endpoint_IsOUTReceived())
    {
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
