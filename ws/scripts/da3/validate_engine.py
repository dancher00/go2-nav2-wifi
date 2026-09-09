"""Gate live TensorRT use on numeric agreement with the reference model."""
import argparse
import hashlib
import json
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('engine')
p.add_argument('benchmark')
p.add_argument('output')
a=p.parse_args()
report=json.loads(Path(a.benchmark).read_text())
assert report['backend']=='trt'
assert len(report['frames'])>=3
for row in report['frames']:
    assert 0<=row['reference_relative_mae']<.01, row
    assert all(0<=v<.02 for v in row['output_relative_l2'].values()), row
out={'engine_sha256':hashlib.sha256(Path(a.engine).read_bytes()).hexdigest(),
     'benchmark':str(Path(a.benchmark).resolve()),'validated_frames':len(report['frames']),
     'meaning':'Numerical agreement only; not metric calibration or mapping validation'}
Path(a.output).write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
