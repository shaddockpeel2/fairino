"""Capture exactly one stationary hand-eye pose and exit."""
import argparse
import json
from pathlib import Path

import rclpy

from .handeye_session import (
    HandEyeCollector,
    Session,
    add_common_arguments,
    validate_options,
    wait_until_ready,
)


def dataset_status(path):
    document = json.loads((path / 'samples.json').read_text())
    samples = document.get('samples', [])
    return {
        'session': str(path),
        'samples': len(samples),
        'last_index': samples[-1]['index'] if samples else None,
        'camera_serial': document.get('camera_serial'),
        'board': document.get('board'),
    }


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Append one stopped-pose ChArUco hand-eye sample, then exit.')
    add_common_arguments(parser)
    parser.add_argument('--session', required=True,
                        help='dataset directory; created on first capture and reused thereafter')
    parser.add_argument('--status', action='store_true',
                        help='show the existing dataset count without using ROS')
    options = parser.parse_args(args)
    validate_options(parser, options)
    session_path = Path(options.session).expanduser().resolve()
    samples_path = session_path / 'samples.json'
    if options.status:
        if not samples_path.is_file():
            raise SystemExit(f'dataset does not exist: {samples_path}')
        print(json.dumps(dataset_status(session_path), indent=2))
        return
    if session_path.exists() and not samples_path.is_file():
        raise SystemExit(f'refusing non-session directory: {session_path}')

    rclpy.init()
    node = HandEyeCollector(options)
    try:
        if not wait_until_ready(node):
            result = {
                'captured': False,
                'reason': 'camera_tf_or_charuco_readiness_timeout',
                'rejects': node.rejects,
            }
            print(json.dumps(result, indent=2))
            raise SystemExit(2)
        session = Session(node, options, session_path, resume=samples_path.is_file())
        success, detail = session.capture()
        if success:
            detail = {
                'index': detail['index'],
                'role': detail['role'],
                'quality': detail['quality'],
                'images': detail['images'],
            }
        result = {
            'captured': success,
            'session': str(session_path),
            'sample_count': len(session.samples),
            'detail': detail,
        }
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if success else 2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
