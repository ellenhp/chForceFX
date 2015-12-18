#ifndef __usb_hid_h__
#define __usb_hid_h__

//------------------------------------------------------------------------------
// Standard control endpoint request types

#define GET_STATUS		0
#define CLEAR_FEATURE		1
#define SET_FEATURE		3
#define SET_ADDRESS		5
#define GET_DESCRIPTOR		6
#define GET_CONFIGURATION	8
#define SET_CONFIGURATION	9
#define GET_INTERFACE		10
#define SET_INTERFACE		11

//------------------------------------------------------------------------------
// HID (human interface device)

#define HID_GET_REPORT		1
#define HID_GET_IDLE		2
#define HID_GET_PROTOCOL	3
#define HID_SET_REPORT		9
#define HID_SET_IDLE		10
#define HID_SET_PROTOCOL	11

//------------------------------------------------------------------------------

#define LSB( n )		( (uint16_t)(n)       & 0xFF)
#define MSB( n )		(((uint16_t)(n) >> 8) & 0xFF)

#define	LVAL( n )		LSB( n ), MSB( n )

#endif // __usb_hid_h__
