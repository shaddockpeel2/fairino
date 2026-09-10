"""Read-only USB identity and topology preflight for the project DaBai DCW2."""

import argparse
import json
import os
from pathlib import Path
import stat


ORBBEC_VENDOR_ID = '2bc5'
DCW2_PRODUCT_IDS = {'0561', '06a0'}


def _read_text(path):
    try:
        return path.read_text(encoding='utf-8').strip()
    except (FileNotFoundError, PermissionError, OSError):
        return ''


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
            **node_details,
        })
    return devices


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
    result = {
        'pass': not failures,
        'expected_model': 'DaBai DCW2',
        'expected_serial_number': options.serial,
        'expected_connection': 'USB2.0',
        'expected_speed_mbps': options.expected_speed_mbps,
        'matching_devices': matching,
        'all_orbbec_devices': devices,
        'failures': failures,
    }
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    print(rendered)
    if options.json_output:
        options.json_output.parent.mkdir(parents=True, exist_ok=True)
        options.json_output.write_text(rendered + '\n', encoding='utf-8')
    return 0 if not failures else 1
