import io
import json
from types import SimpleNamespace
from fr3c_target_follow.preview_audit import PreviewAudit


def test_report_with_target_loss_is_saved_as_valid_json(tmp_path):
    sample=dict(status='stale_or_missing_target',motion_commands_enabled=False,
                orientation_control_enabled=False,velocity_m_s=[0.,0.,0.],audit_receive_s=1.)
    fixture=SimpleNamespace(output=tmp_path,states=[sample],robot=[],flange=[],targets=[],stream=io.StringIO())
    report=PreviewAudit.report(fixture,1.)
    saved=json.loads((tmp_path/'report.json').read_text())
    assert saved['preview_status_counts']=={'stale_or_missing_target':1}
    assert saved['nonpreview_nonzero_velocity_samples']==0
    assert saved['base_target_samples']==0
    assert not report['motion_commands_enabled']
