# P2 field acceptance summary

Date: 2026-09-16  
Result: PASS — stage 2 closed.

Verified successful evidence runs:

- `evidence/P5/runs/1789528733375495964`: red `pick_hold`, `complete_holding`.
- `evidence/P5/runs/1789528817328744269`: red `pick_sort`, `complete_sorted_home`.
- `evidence/P5/runs/1789529592643146960`: blue `pick_sort`, `complete_sorted_home`.
- `evidence/P5/runs/1789529752350922568`: yellow `pick_sort`, `complete_sorted_home`.
- `evidence/P5/runs/1789529835161501641`: blue `pick_sort`, `complete_sorted_home`.

The red hold run was followed by operator release acknowledgement and a
successful red sort run. The final observed task state was an inactive blue
`complete_sorted_home` result.

All recorded arrival errors were inside the configured 3 mm position and
0.034907 rad orientation tolerances. Three earlier gripper pre-open failures
stopped before arm motion and later recovered through a new explicit goal.

Limitations retained in the technical report:

- Gripper closure is open-loop; success does not prove a sensed grip.
- Some successful routes exceeded the joint6 soft travel preference but passed
  hard joint, collision, full-path, and arrival checks.
- No green physical cycle is included in this acceptance evidence.

Technical report:
`docs/voice/FR3C离线语音抓取P2颜色任务Action技术报告-20260916.md`
