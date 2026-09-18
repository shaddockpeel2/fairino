#!/usr/bin/env python3
"""Join P7 input and P6 output traces by request_id (software timing only)."""
import argparse
import json
from collections import defaultdict
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('traces', type=Path, nargs='+')
args = parser.parse_args()
groups = defaultdict(dict)
for path in args.traces:
    for line in path.read_text().splitlines():
        record = json.loads(line)
        rid = record.get('request_id')
        if rid:
            groups[rid][record.get('event')] = record
for rid, events in groups.items():
    final = events.get('asr_final')
    if not final:
        continue
    end = final.get('speech_end_monotonic_ns')
    first = events.get('first_valid_pcm_write', {}).get('monotonic_ns')
    final_ns = final['monotonic_ns']
    print(json.dumps(dict(request_id=rid, reason=final.get('reason'),
                         speech_end_to_final_s=(final_ns-end)/1e9 if end else None,
                         final_to_first_device_write_s=(first-final_ns)/1e9 if first else None,
                         acoustic_first_sound_s=None), ensure_ascii=False))
