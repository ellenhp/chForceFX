import usb.core
import usb.util
import usb.backend
import sys
from datetime import datetime
import pickle
import subprocess
import time
from time import sleep

def fromSChar(schar):
    if (schar & 0x80) != 0:
        numberPart = (~schar) & 0x7F
        return -(numberPart + 1)
    return schar

def main():
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

    subprocess.Popen(["ffplay", "../data/halfspeed.mp3"])

    waypoints = []

    startTime = datetime.now()

    try:
        while True:
            data = device.read(0x81, 5, timeout=100)
            # print data
            x, y = fromSChar(data[1]), fromSChar(data[2])
            print x, y
            elapsedTime = (datetime.now() - startTime).total_seconds()
            waypoints.append([elapsedTime/2.0, x, y])
            sleep(0.005)
    except KeyboardInterrupt:
        print 'bye!'

    filename = time.strftime("%Y%m%d-%H%M%S.jsdance")
    with file(filename, 'w') as f:
        pickle.dump(waypoints, f)

    usb.util.release_interface(device, intf.bInterfaceNumber)
    device.attach_kernel_driver(intf.bInterfaceNumber)

main()
