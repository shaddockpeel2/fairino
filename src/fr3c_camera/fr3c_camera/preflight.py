"""Read-only USB identity and topology preflight for the project DaBai DCW2."""

import argparse
import ctypes
import ctypes.util
import json
import os
from pathlib import Path
import stat
import time


ORBBEC_VENDOR_ID = '2bc5'
DCW2_PRODUCT_IDS = {'0561', '06a0'}
DCW2_PRODUCT_IDS_INT = {int(product_id, 16) for product_id in DCW2_PRODUCT_IDS}


class _UsbDeviceDescriptor(ctypes.Structure):
    _fields_ = [
        ('bLength', ctypes.c_uint8), ('bDescriptorType', ctypes.c_uint8),
        ('bcdUSB', ctypes.c_uint16), ('bDeviceClass', ctypes.c_uint8),
        ('bDeviceSubClass', ctypes.c_uint8), ('bDeviceProtocol', ctypes.c_uint8),
        ('bMaxPacketSize0', ctypes.c_uint8), ('idVendor', ctypes.c_uint16),
        ('idProduct', ctypes.c_uint16), ('bcdDevice', ctypes.c_uint16),
        ('iManufacturer', ctypes.c_uint8), ('iProduct', ctypes.c_uint8),
        ('iSerialNumber', ctypes.c_uint8), ('bNumConfigurations', ctypes.c_uint8),
    ]


def _read_text(path):
    try:
        return path.read_text(encoding='utf-8').strip()
    except (FileNotFoundError, PermissionError, OSError):
        return ''


def _external_usb_hubs(device_path):
    """Return external hub ancestors, closest to the camera first."""
    hubs = []
    try:
        parents = device_path.resolve().parents
    except OSError:
        return hubs
    for parent in parents:
        if parent.name.startswith('usb'):
            continue
        if _read_text(parent / 'bDeviceClass') == '09':
            hubs.append(parent.name)
    return hubs


def scan_orbbec_devices(sys_usb_root=Path('/sys/bus/usb/devices')):
    """Return Orbbec USB functions without opening or changing the camera."""
    devices = []
    for device_path in sorted(sys_usb_root.glob('*')):
        if _read_text(device_path / 'idVendor').lower() != ORBBEC_VENDOR_ID:
            continue
        busnum = _read_text(device_path / 'busnum')
        devnum = _read_text(device_path / 'devnum')
        node_details = {}
        if busnum.isdigit() and devnum.isdigit():
            device_node = Path('/dev/bus/usb') / f'{int(busnum):03d}' / f'{int(devnum):03d}'
            try:
                node_stat = device_node.stat()
                node_details = {
                    'device_node': str(device_node),
                    'device_node_exists': True,
                    'device_node_mode': stat.filemode(node_stat.st_mode),
                    'device_node_readable': os.access(device_node, os.R_OK),
                    'device_node_writable': os.access(device_node, os.W_OK),
                }
            except (FileNotFoundError, PermissionError, OSError):
                node_details = {
                    'device_node': str(device_node),
                    'device_node_exists': False,
                    'device_node_mode': '',
                    'device_node_readable': False,
                    'device_node_writable': False,
                }
        devices.append({
            'sysfs_name': device_path.name,
            'product_id': _read_text(device_path / 'idProduct').lower(),
            'product': _read_text(device_path / 'product'),
            'serial_number': _read_text(device_path / 'serial'),
            'speed_mbps': _read_text(device_path / 'speed'),
            'busnum': busnum,
            'devnum': devnum,
            'authorized': _read_text(device_path / 'authorized'),
            'power_control': _read_text(device_path / 'power' / 'control'),
            'external_usb_hubs': _external_usb_hubs(device_path),
            **node_details,
        })
    return devices


def probe_control_transfers():
    """Perform a real USB serial-descriptor read, not a cached sysfs lookup.

    A wedged DaBai can remain visible to ``lsusb`` while every control transfer
    times out.  The ROS driver then loops on device discovery.  This probe
    detects that state before the driver is launched.
    """
    library_name = ctypes.util.find_library('usb-1.0')
    if not library_name:
        return [], 'libusb-1.0 is not installed'
    try:
        libusb = ctypes.CDLL(library_name)
    except OSError as error:
        return [], f'failed to load libusb-1.0: {error}'

    libusb.libusb_init.argtypes = [ctypes.POINTER(ctypes.c_void_p)]
    libusb.libusb_init.restype = ctypes.c_int
    libusb.libusb_exit.argtypes = [ctypes.c_void_p]
    libusb.libusb_get_device_list.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(ctypes.POINTER(ctypes.c_void_p))]
    libusb.libusb_get_device_list.restype = ctypes.c_ssize_t
    libusb.libusb_free_device_list.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_int]
    libusb.libusb_get_device_descriptor.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(_UsbDeviceDescriptor)]
    libusb.libusb_get_device_descriptor.restype = ctypes.c_int
    libusb.libusb_open.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)]
    libusb.libusb_open.restype = ctypes.c_int
    libusb.libusb_close.argtypes = [ctypes.c_void_p]
    libusb.libusb_get_string_descriptor_ascii.argtypes = [
        ctypes.c_void_p, ctypes.c_uint8, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
    libusb.libusb_get_string_descriptor_ascii.restype = ctypes.c_int
    libusb.libusb_set_option.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]

    context = ctypes.c_void_p()
    init_rc = libusb.libusb_init(ctypes.byref(context))
    if init_rc != 0:
        return [], f'libusb initialization failed: {init_rc}'

    # LIBUSB_OPTION_LOG_LEVEL = 0.  The timeout is controlled internally by
    # libusb for descriptor reads; setting quiet logging keeps JSON parseable.
    libusb.libusb_set_option(context, 0, 0)
    device_list = ctypes.POINTER(ctypes.c_void_p)()
    count = libusb.libusb_get_device_list(context, ctypes.byref(device_list))
    if count < 0:
        libusb.libusb_exit(context)
        return [], f'libusb device enumeration failed: {count}'

    probes = []
    try:
        for index in range(count):
            descriptor = _UsbDeviceDescriptor()
            rc = libusb.libusb_get_device_descriptor(
                device_list[index], ctypes.byref(descriptor))
            if rc != 0 or descriptor.idVendor != int(ORBBEC_VENDOR_ID, 16):
                continue
            if descriptor.idProduct not in DCW2_PRODUCT_IDS_INT:
                continue
            handle = ctypes.c_void_p()
            open_rc = libusb.libusb_open(device_list[index], ctypes.byref(handle))
            probe = {
                'product_id': f'{descriptor.idProduct:04x}',
                'status': 'OPEN_FAILED' if open_rc != 0 else 'UNKNOWN',
                'result_code': open_rc,
            }
            if open_rc == 0:
                buffer = (ctypes.c_ubyte * 256)()
                started = time.monotonic()
                serial_rc = libusb.libusb_get_string_descriptor_ascii(
                    handle, descriptor.iSerialNumber, buffer, len(buffer))
                elapsed_ms = (time.monotonic() - started) * 1000.0
                libusb.libusb_close(handle)
                probe['elapsed_ms'] = round(elapsed_ms, 1)
                probe['result_code'] = serial_rc
                if serial_rc > 0:
                    probe['status'] = 'OK'
                    probe['serial_number'] = bytes(buffer[:serial_rc]).decode(
                        'utf-8', errors='replace')
                else:
                    probe['status'] = 'HUNG_OR_UNREADABLE'
            probes.append(probe)
    finally:
        libusb.libusb_free_device_list(device_list, 1)
        libusb.libusb_exit(context)
    return probes, None


def evaluate_control_probes(probes, serial_number, probe_error=None):
    failures = []
    if probe_error:
        failures.append(probe_error)
        return failures
    products = {probe['product_id'] for probe in probes}
    missing = sorted(DCW2_PRODUCT_IDS - products)
    if missing:
        failures.append('control probe did not find USB product(s): ' + ', '.join(missing))
    for probe in probes:
        if probe['status'] != 'OK':
            failures.append(
                f"USB control transfer failed for 2bc5:{probe['product_id']}: "
                f"{probe['status']} (rc={probe['result_code']})")
        elif probe.get('serial_number') != serial_number:
            failures.append(
                f"USB control probe serial mismatch for 2bc5:{probe['product_id']}: "
                f"{probe.get('serial_number') or '<empty>'}")
    return failures


def stability_warnings(devices):
    warnings = []
    autosuspend = sorted(device['sysfs_name'] for device in devices
                         if device.get('power_control') == 'auto')
    if autosuspend:
        warnings.append(
            'USB autosuspend is enabled for: ' + ', '.join(autosuspend) +
            '; install/refresh scripts/camera/99-fr3c-orbbec-usb.rules')
    hub_paths = sorted({tuple(device.get('external_usb_hubs', ())) for device in devices})
    if any(len(path) > 1 for path in hub_paths):
        warnings.append(
            'camera is behind cascaded external USB hubs; DaBai DCW2 requires '
            'a stable USB 2.0 host supply rated at least 5 V / 1.5 A')
    return warnings


def evaluate_devices(devices, serial_number, expected_speed_mbps='480'):
    """Reject a missing/wrong camera before the ROS driver picks a device."""
    matching = [device for device in devices if device['serial_number'] == serial_number]
    unexpected = [device for device in devices if device['serial_number'] != serial_number]
    products = {device['product_id'] for device in matching}
    failures = []
    if not matching:
        failures.append(f'camera serial {serial_number} was not found')
    if unexpected:
        identities = sorted({device['serial_number'] or device['sysfs_name'] for device in unexpected})
        failures.append('unexpected Orbbec device(s) present: ' + ', '.join(identities))
    missing_products = sorted(DCW2_PRODUCT_IDS - products)
    if missing_products:
        failures.append('missing DaBai DCW2 USB product(s): ' + ', '.join(missing_products))
    unauthorized = [device['sysfs_name'] for device in matching
                    if device['authorized'] not in ('', '1')]
    if unauthorized:
        failures.append('unauthorized USB device(s): ' + ', '.join(unauthorized))
    inaccessible = [device['sysfs_name'] for device in matching
                    if device.get('device_node') and not
                    (device.get('device_node_readable') and device.get('device_node_writable'))]
    if inaccessible:
        failures.append('USB device node(s) are not readable/writable: ' + ', '.join(inaccessible))
    wrong_speed = [f"{device['sysfs_name']}={device['speed_mbps']}"
                   for device in matching
                   if device['speed_mbps'] and device['speed_mbps'] != expected_speed_mbps]
    if wrong_speed:
        failures.append(f'expected USB 2.0 ({expected_speed_mbps} Mbps), got ' + ', '.join(wrong_speed))
    return matching, failures


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', default='CH82C5201C9')
    parser.add_argument('--expected-speed-mbps', default='480')
    parser.add_argument('--json-output', type=Path)
    options = parser.parse_args(args)
    devices = scan_orbbec_devices()
    matching, failures = evaluate_devices(devices, options.serial, options.expected_speed_mbps)
    control_probes, probe_error = probe_control_transfers()
    failures.extend(evaluate_control_probes(control_probes, options.serial, probe_error))
    result = {
        'pass': not failures,
        'expected_model': 'DaBai DCW2',
        'expected_serial_number': options.serial,
        'expected_connection': 'USB2.0',
        'expected_speed_mbps': options.expected_speed_mbps,
        'matching_devices': matching,
        'all_orbbec_devices': devices,
        'control_transfer_probes': control_probes,
        'warnings': stability_warnings(matching),
        'failures': failures,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if options.json_output:
        options.json_output.parent.mkdir(parents=True, exist_ok=True)
        options.json_output.write_text(rendered + '\n', encoding='utf-8')
    return 0 if not failures else 1
