def main():
    import usb.core
    import usb.util
    import usb.backend
    import sys

    device = usb.core.find(idVendor=0x03eb, idProduct=0x2056)

    if device is None:
        raise ValueError('Device not found')
    else:
        print "Device found"

    # device.set_configuration()

    for cfg in device:
        for intf in cfg:
            if device.is_kernel_driver_active(intf.bInterfaceNumber):
                try:
                    device.detach_kernel_driver(intf.bInterfaceNumber)
                    usb.util.claim_interface(device, intf.bInterfaceNumber)
                except usb.core.USBError as e:
                    sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))

    device.ctrl_transfer(0x21, 0x09, 0x0309, 0x0, timeout=100)

    usb.util.release_interface(device, intf.bInterfaceNumber)
    device.attach_kernel_driver(intf.bInterfaceNumber)

main()
