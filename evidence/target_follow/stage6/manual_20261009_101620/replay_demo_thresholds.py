"""Replay recorded orientation with looser demo thresholds, without ROS/SDK."""
from collections import Counter
import json
from pathlib import Path

import numpy as np

from fr3c_target_follow.follow_geometry import rotation_from_quaternion
from fr3c_target_follow.orientation_quality import OrientationGate, measurement_problem, stamp_ns


ROOT = Path(__file__).resolve().parent
OUT = ROOT/'demo_threshold_analysis'


def reclassify(quality, margin):
    """Same PnP candidates/selection; only reconstruct ambiguity classification."""
    quality = dict(quality)
    if quality['status'] != 'valid':
        return quality
    errors = quality['candidate_errors_px']
    separation = quality['candidate_rotation_distance_deg']
    ambiguous = (quality['solver'] == 'ITERATIVE_FALLBACK' or
                 (len(errors) > 1 and errors[1]-errors[0] < margin and separation > 5.))
    quality['orientation_ambiguous'] = ambiguous
    quality['orientation_measurement_valid'] = (not ambiguous and
        (quality['selected_candidate_index'] == 0 or (separation is not None and separation <= 5.)))
    return quality


def replay(rows, report, margin, confirmation):
    qualities = {stamp_ns(row['stamp']): row for row in rows
                 if row['kind'] == 'quality' and row.get('stamp') is not None}
    gate = OrientationGate(confirmation_frames=confirmation)
    counts, run_lengths, run_spans, run = Counter(), [], [], []
    ambiguous_source = {'orientation_ambiguous', 'orientation_measurement_rejected', 'target_not_found'}
    for row in rows:
        if row['kind'] != 'observation':
            continue
        if row.get('position_m') is None:
            if row['status'] in ambiguous_source:
                quality = qualities.get(stamp_ns(row['stamp']))
                if quality is None:
                    raise ValueError('Missing recorded quality for early invalidation')
                if measurement_problem(reclassify(quality, margin)) is not None:
                    gate.confirmed = 0
            else:
                gate.reset()
            continue
        result = gate.observe(stamp_ns(row['stamp']), rotation_from_quaternion(row['quaternion_xyzw']),
                              measurement_problem(reclassify(row['quality'], margin)))
        counts[result['status']] += 1
        if result['valid_orientation']:
            run.append(row['audit_receive_s'])
        elif run:
            run_lengths.append(len(run))
            run_spans.append(run[-1]-run[0])
            run = []
    if run:
        run_lengths.append(len(run))
        run_spans.append(run[-1]-run[0])
    if margin == .15 and confirmation == 5:
        assert dict(counts) == report['paired_status_counts'], (counts, report['paired_status_counts'])
    return dict(ambiguity_margin_px=margin, confirmation_frames=confirmation,
                status_counts=dict(counts), available_fraction=counts['valid']/sum(counts.values()),
                longest_valid_run_frames=max(run_lengths, default=0),
                longest_valid_run_span_s=max(run_spans, default=0.),
                note='Run spans are first-to-last sample; higher availability does not establish correctness. '
                     'Candidates and measurements are unchanged; no ROS pairing/latency or robot execution simulation.')


def main():
    OUT.mkdir(exist_ok=True)
    results = {}
    for case in ('static', 'static_more_pixels', 'target_rotation'):
        rows = [json.loads(line) for line in (ROOT/case/'samples.jsonl').read_text().splitlines()]
        report = json.loads((ROOT/case/'report.json').read_text())
        results[case] = [replay(rows, report, margin, confirmation)
                         for margin, confirmation in ((.15, 5), (.15, 2), (.10, 2), (.05, 2))]
    (OUT/'comparison.json').write_text(json.dumps(dict(
        cases=results, absolute_pose_accuracy_verified=False, hardware_commands_sent=0,
        unchanged_parameters=dict(max_angular_speed_deg_s=60., jitter_allowance_deg=3.,
                                  timeout_s=.35, max_reprojection_error_px=2., continuity_margin_px=.20),
        limitations='Uses selected candidates recorded under the original configuration. Changing ambiguity '
                    'classification does not change PnP selection, but real scheduling/quality arrival may differ.'),
        ensure_ascii=False, indent=2)+'\n')
    for case, variants in results.items():
        print(case)
        for row in variants:
            print(f"  margin={row['ambiguity_margin_px']:.2f} confirm={row['confirmation_frames']} "
                  f"available={row['available_fraction']:.2%} longest_run={row['longest_valid_run_span_s']:.2f}s "
                  f"counts={row['status_counts']}")


if __name__ == '__main__':
    main()
