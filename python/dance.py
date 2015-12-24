import usb.core
import usb.util
import usb.backend
import sys
from time import sleep
from datetime import datetime

import pickle

from scipy import interpolate

import subprocess

def toSChar(num):
    num = min(127, max(-127, int(num)))
    if num < 0:
        return 0x80 | (num & 0x7F)
    else:
        return num

def main():

    device = usb.core.find(idVendor=0x03eb, idProduct=0x2056)

    if device is None:
        raise ValueError('Device not found')
    else:
        print "Device found"

    for cfg in device:
        for intf in cfg:
            if device.is_kernel_driver_active(intf.bInterfaceNumber):
                try:
                    device.detach_kernel_driver(intf.bInterfaceNumber)
                    usb.util.claim_interface(device, intf.bInterfaceNumber)
                except usb.core.USBError as e:
                    sys.exit("Could not detatch kernel driver from interface({0}): {1}".format(intf.bInterfaceNumber, str(e)))

    data = pickle.load(file('../data/test.jsdance', 'r'))

    data = sorted(data, key=lambda point: data[0])

    times = [point[0] for point in data]
    waypoints = [point[1:3] for point in data]

    print times
    print waypoints

    waypointGenerator = interpolate.interp1d(times, waypoints, bounds_error=False, fill_value=[0,0], axis=0)

    subprocess.Popen(["ffplay", "../data/test.mp3"])

    pidParams = [8, 0, 3, 0]
    device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00] + pidParams, timeout=100)

    # sleep (0.3)

    startTime = datetime.now()

    try:
        while True:
            timeSinceStart = (datetime.now() - startTime).total_seconds()
            position = 1.5 * waypointGenerator(timeSinceStart)
            device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [toSChar(position[0]), toSChar(position[1])] + pidParams, timeout=100)
            sleep(0.005)

    except KeyboardInterrupt:
        print 'cleaning up'
        device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00, 0, 0, 0], timeout=100)

        usb.util.release_interface(device, intf.bInterfaceNumber)
        device.attach_kernel_driver(intf.bInterfaceNumber)

main()
