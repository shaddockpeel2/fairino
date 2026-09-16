import importlib.util
import json
from pathlib import Path
import sys

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts/voice'
sys.path.insert(0, str(SCRIPTS))
from p1_summarize import summarize


def test_pid_reuse_does_not_create_false_cpu_peak(tmp_path):
    rows=[]
    for timestamp, start, ticks in ((1000000000, 10, 1000), (2000000000, 11, 4)):
        rows.append({'monotonic_ns':timestamp,'memory':{'MemAvailable_kib':2048,'SwapTotal_kib':0,'SwapFree_kib':0},'npu':{'raw':None},'processes':[{'pid':8,'start_ticks':start,'cpu_ticks':ticks,'rss_kib':1024,'component':'qwen','comm':'worker'}]})
    (tmp_path/'resources.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    result=summarize(tmp_path)
    assert result['components']['qwen']['peak_process_cpu_percent']==0
    assert result['model_startup_s'] is None
    assert result['npu_available'] is False


def test_fixed_text_lengths():
    path=SCRIPTS.parents[1]/'config/voice/p1_texts.json'
    values=json.loads(path.read_text())
    assert [v['characters'] for v in values]==[10,30,60,120]
    assert all(len(v['text'])==v['characters'] for v in values)
