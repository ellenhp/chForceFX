#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

// Define: ENABLE_JOYSTICK_SERIAL
//	When defined, includes USB COM serial port to the device
//	in addition to joystick.
#define ENABLE_JOYSTICK_SERIAL

/* Includes: */
#include <LUFA/Drivers/USB/USB.h>

#include <avr/pgmspace.h>

/* Type Defines: */
/** Type define for the device configuration descriptor structure. This must be defined in the
 *  application code, as the configuration descriptor contains several sub-descriptors which
 *  vary between devices, and which describe the device's usage to the host.
 */

typedef struct
{
	USB_Descriptor_Configuration_Header_t Config;

	// Joystick HID Interface
	USB_Descriptor_Interface_t            HID_Interface;

	// Joystick stuff
	USB_HID_Descriptor_HID_t              HID_JoystickHID;
	USB_Descriptor_Endpoint_t             HID_ReportOUTEndpoint;
    USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;

} USB_Descriptor_Configuration_t;

/** Endpoint number of the Joystick HID reporting IN endpoint. */
#define JOYSTICK_EPNUM 1

/** Size in bytes of the Joystick HID reporting IN endpoint. */
#define JOYSTICK_EPSIZE 64

/** Descriptor header type value, to indicate a HID class HID descriptor. */
#define DTYPE_HID 0x21

/** Descriptor header type value, to indicate a HID class HID report descriptor. */
#define DTYPE_Report 0x22

/** Endpoint number of the Joystick HID reporting IN endpoint. */
#define FFB_EPNUM 2

/** Size in bytes of the Joystick FFB HID reporting OUT endpoint. */
#define FFB_EPSIZE 64

// Serial device stuff

#define CDC1_TX_EPNUM 3
#define CDC1_RX_EPNUM 4
#define CDC1_NOTIFICATION_EPNUM 5

/** Size in bytes of the CDC device-to-host notification IN endpoints. */
#define CDC_NOTIFICATION_EPSIZE 8

/** Size in bytes of the CDC data IN and OUT endpoints. */
#define CDC_TXRX_EPSIZE 16

/* Function Prototypes: */
uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
                                    const uint8_t wIndex,
                                    const void** const DescriptorAddress)
                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif
