from fr3c_camera.preflight import evaluate_devices


def device(product_id, speed='480', serial='CH82C5201C9'):
    return {
        'sysfs_name': '3-1', 'product_id': product_id, 'product': 'DaBai DCW2',
        'serial_number': serial, 'speed_mbps': speed, 'devnum': '4',
        'authorized': '1',
    }


def test_accepts_expected_dcw2_usb_functions():
    matching, failures = evaluate_devices([device('06a0'), device('0561')], 'CH82C5201C9')
    assert len(matching) == 2
    assert failures == []


def test_rejects_missing_rgb_function_and_wrong_speed():
    _, failures = evaluate_devices([device('06a0', speed='5000')], 'CH82C5201C9')
    assert any('missing DaBai DCW2' in failure for failure in failures)
    assert any('expected USB 2.0' in failure for failure in failures)


def test_rejects_another_orbbec_camera():
    _, failures = evaluate_devices(
        [device('06a0'), device('0561'), device('06a0', serial='OTHER')],
        'CH82C5201C9')
    assert any('unexpected Orbbec' in failure for failure in failures)
