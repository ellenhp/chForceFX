import usb.core
import usb.util
import usb.backend
import sys
from time import sleep
from datetime import datetime

from pypid.backend import Backend

def toSChar(num):
    num = min(127, max(-127, num))
    if num < 0:
        return 0x80 | (num & 0x7F)
    else:
        return num

def testAxis(targetPosition, currentPosition, state):
    if state is None:
        state = (currentPosition, 0)

    velocity = [currentPosition[0] - state[0][0], currentPosition[1] - state[0][1]]
    error = [targetPosition[0] - currentPosition[0], targetPosition[1] - currentPosition[1]]

    if state[1] < 15:
        return (0, 127), (currentPosition, state[1] + 1)
    else:
        print currentPosition[1]
        return (0, 0), (currentPosition, state[1] + 1)

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

    testParams = [8, 0, 5, 0]

    stabilizingTime = 2

    startPositions = [
        [-127, 0],
        [-100, -100],
        [100, 0],
        [0, 100],
    ]

    try:
        for startPos in startPositions:
            device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [toSChar(startPos[0]), toSChar(startPos[1]), 8, 0, 1, 0], timeout=100)

            sleep(1)

            device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00] + testParams, timeout=100)

            sleep(stabilizingTime)
    except:
        print 'cleaning up'
        device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00, 0, 0, 0], timeout=100)

        usb.util.release_interface(device, intf.bInterfaceNumber)
        device.attach_kernel_driver(intf.bInterfaceNumber)

    device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00] + testParams, timeout=100)
    # print 'cleaning up'
    # device.ctrl_transfer(0x21, 0x09, 0x030F, 0x00, [0x00, 0x00, 0, 0, 0], timeout=100)
main()
