#!/usr/bin/env python3
"""Check the recorded five-UAV production Candidate scenario."""
import json,sys
from pathlib import Path
report=json.loads(Path(sys.argv[1]).read_text())
ids={str(x) for x in range(1,6)}
assert set(report['samples'])==ids and all(n>100 for n in report['samples'].values())
assert report['fault_count']==0, report['fault_count']
assert report['minimum_pair_distance_m']>=1.5, report['minimum_pair_distance_m']
assert all(m==[2111] for m in report['setpoint_masks'].values()), 'Acceleration-only MAVROS mask required'
assert len(report['commands'])==15, 'Expected all three stages (5 + 3/2 + 5 commands)'
assert {c['style'] for c in report['commands']}=={'normal','smooth','aggressive'}
assert all(c['configuration_id']=='paper-current-v11-c0-f-frozen' for c in report['commands'])
parallel=[c['duration'] for c in report['commands'] if c['task'] in (2,3)]
assert len(parallel)==5 and max(parallel)-min(parallel)<1e-5
assert set(report['final'])==ids
assert all(s['ready'] and s['stable'] and s['armed'] and s['offboard'] and s['position_error']<.4 for s in report['final'].values())
print('PASS: five-UAV Candidate sequence, synchronized parallel profiles, acceleration masks, safety distance and completion')
