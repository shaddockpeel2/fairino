from fr3c_camera.preflight import evaluate_control_probes, stability_warnings


def test_control_probe_rejects_a_visible_but_hung_usb_function():
    probes = [
        {'product_id': '0561', 'status': 'OK', 'result_code': 12,
         'serial_number': 'CH82C5201C9'},
        {'product_id': '06a0', 'status': 'HUNG_OR_UNREADABLE', 'result_code': -7},
    ]

    failures = evaluate_control_probes(probes, 'CH82C5201C9')

    assert failures == [
        'USB control transfer failed for 2bc5:06a0: '
        'HUNG_OR_UNREADABLE (rc=-7)']


def test_control_probe_requires_both_dcw2_usb_functions():
    probes = [
        {'product_id': '06a0', 'status': 'OK', 'result_code': 12,
         'serial_number': 'CH82C5201C9'},
    ]

    failures = evaluate_control_probes(probes, 'CH82C5201C9')

    assert failures == ['control probe did not find USB product(s): 0561']


def test_stability_warnings_cover_autosuspend_and_cascaded_hubs():
    devices = [{
        'sysfs_name': '5-1.3.3',
        'power_control': 'auto',
        'external_usb_hubs': ['5-1.3', '5-1'],
    }]

    warnings = stability_warnings(devices)

    assert 'USB autosuspend is enabled' in warnings[0]
    assert 'cascaded external USB hubs' in warnings[1]
