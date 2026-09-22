#!/usr/bin/env python3
"""Summarize recorded observations without declaring physical acceptance."""
import collections
import json
import math
import sys
from pathlib import Path

for name in sys.argv[1:]:
    data = json.loads(Path(name).read_text())
    feedback = data['feedback']
    commands = data['commands']
    states = data['status']
    if states:
        run_id = states[-1][1]['run_id']
        states = [s for s in states if s[1]['run_id'] == run_id]
    nonzero = [c for c in commands if math.hypot(c[1], c[2]) > 1e-9]
    print(json.dumps(dict(file=name, result=data['result'],
        final_control_state=data['final_control_state'],
        phase_counts=dict(collections.Counter(f['phase'] for f in feedback)),
        first_error_px=feedback[0]['error_px'] if feedback else None,
        last_error_px=feedback[-1]['error_px'] if feedback else None,
        nonzero_commands=len(nonzero),
        max_command_mps=max((math.hypot(c[1], c[2]) for c in commands), default=0),
        max_non_xy_command=max((max(abs(v) for v in c[3:]) for c in commands), default=0),
        max_measured_xy_m=max((s[1].get('measured_relative_xy_m', 0) for s in states), default=0),
        servo_states=sorted(set(s[1] for s in data['servo_states']))), ensure_ascii=False))
