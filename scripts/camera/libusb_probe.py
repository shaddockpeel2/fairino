#!/usr/bin/env python3
"""Live-communication probe for the project DaBai DCW2 USB camera.

Unlike `lsusb -v`, this performs real control transfers against the device so a
wedged/hung camera is detected instead of reporting cached descriptors.

Usage:
    python3 libusb_probe.py [serial ...]
"""

import ctypes
import ctypes.util
import sys
import time

ORBBEC_VID = 0x2BC5
DCW2_PIDS = (0x06A0, 0x0561)

LIBUSB_SUCCESS = 0
LIBUSB_ERROR_TIMEOUT = -7
LIBUSB_ERROR_NO_DEVICE = -4
LIBUSB_ERROR_ACCESS = -3

LIBUSB_ENDPOINT_IN = 0x80
LIBUSB_DT_STRING = 0x03
LIBUSB_RECIPIENT_DEVICE = 0x00
LIBUSB_REQUEST_GET_DESCRIPTOR = 0x06

# Short deadline: a healthy USB 2.0 control transfer completes in milliseconds.
TRANSFER_TIMEOUT_MS = 1500


def load_libusb():
    name = ctypes.util.find_library('usb-1.0') or 'libusb-1.0.so.0'
    lib = ctypes.CDLL(name)
    lib.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    lib.libusb_init.restype = ctypes.c_int
    lib.libusb_get_device_list.argtypes = [ctypes.c_void_p,
                                           ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
    lib.libusb_get_device_list.restype = ctypes.c_ssize_t
    lib.libusb_get_device_descriptor.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
    lib.libusb_control_transfer.argtypes = [
        ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16,
        ctypes.c_uint16, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_uint16, ctypes.c_uint]
    lib.libusb_control_transfer.restype = ctypes.c_int
    lib.libusb_close.argtypes = [ctypes.c_void_p]
    return lib


class DeviceDescriptor(ctypes.Structure):
    _fields_ = [
        ('bLength', ctypes.c_uint8), ('bDescriptorType', ctypes.c_uint8),
        ('bcdUSB', ctypes.c_uint16), ('bDeviceClass', ctypes.c_uint8),
        ('bDeviceSubClass', ctypes.c_uint8), ('bDeviceProtocol', ctypes.c_uint8),
        ('bMaxPacketSize0', ctypes.c_uint8), ('idVendor', ctypes.c_uint16),
        ('idProduct', ctypes.c_uint16), ('bcdDevice', ctypes.c_uint16),
        ('iManufacturer', ctypes.c_uint8), ('iProduct', ctypes.c_uint8),
        ('iSerialNumber', ctypes.c_uint8), ('bNumConfigurations', ctypes.c_uint8),
    ]


def read_string(lib, handle, index, langid=0x0409):
    """Issue a real GET_DESCRIPTOR(string) control transfer."""
    if not index:
        return ''
    buf = (ctypes.c_ubyte * 255)()
    transferred = lib.libusb_control_transfer(
        handle,
        LIBUSB_ENDPOINT_IN | LIBUSB_RECIPIENT_DEVICE,
        LIBUSB_REQUEST_GET_DESCRIPTOR,
        (LIBUSB_DT_STRING << 8) | index,
        langid, buf, 255, TRANSFER_TIMEOUT_MS)
    if transferred < 2:
        return None
    raw = bytes(buf[:transferred])
    return raw[2:2 + raw[0] - 2].decode('utf-16-le', errors='replace')


def probe(lib, dev):
    desc = DeviceDescriptor()
    lib.libusb_get_device_descriptor(dev, ctypes.byref(desc))
    if desc.idVendor != ORBBEC_VID or desc.idProduct not in DCW2_PIDS:
        return None

    handle = ctypes.c_void_p()
    rc = lib.libusb_open(dev, ctypes.byref(handle))
    result = {
        'product_id': f'0x{desc.idProduct:04x}',
        'open_rc': rc,
        'control_transfer': None,
    }
    if rc != LIBUSB_SUCCESS:
        result['control_transfer'] = {
            'status': 'OPEN_FAILED', 'rc': rc,
            'detail': {LIBUSB_ERROR_ACCESS: 'permission denied',
                       LIBUSB_ERROR_NO_DEVICE: 'device gone'}.get(rc, 'error'),
        }
        return result

    start = time.monotonic()
    serial = read_string(lib, handle, desc.iSerialNumber)
    elapsed_ms = (time.monotonic() - start) * 1000
    lib.libusb_close(handle)

    if serial is None:
        result['control_transfer'] = {
            'status': 'HUNG', 'elapsed_ms': round(elapsed_ms, 1),
            'detail': 'no data returned within timeout - device firmware not answering',
        }
    else:
        result['control_transfer'] = {
            'status': 'OK', 'elapsed_ms': round(elapsed_ms, 1),
            'serial_number': serial,
        }
    return result


def main(argv):
    lib = load_libusb()
    ctx = ctypes.c_void_p()
    lib.libusb_init(ctypes.byref(ctx))
    devices = ctypes.POINTER(ctypes.c_void_p)()
    count = lib.libusb_get_device_list(ctx, ctypes.byref(devices))

    found = []
    for index in range(count):
        info = probe(lib, devices[index])
        if info:
            found.append(info)

    if not found:
        print('NO ORBBEC DaBai DCW2 DEVICE FOUND ON USB')
        return 1

    healthy = 0
    for info in found:
        transfer = info['control_transfer']
        print(f"2bc5:{info['product_id'][2:]}  status={transfer['status']}  "
              f"elapsed={transfer.get('elapsed_ms')}ms  "
              f"{transfer.get('serial_number') or transfer.get('detail', '')}")
        if transfer['status'] == 'OK':
            healthy += 1
    print(f'RESULT: {healthy}/{len(found)} Orbbec USB function(s) answering control transfers')
    return 0 if healthy == len(found) else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
