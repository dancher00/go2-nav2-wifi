"""Fix one camera input shape and fold constants using standard ONNX ops."""
import argparse
from pathlib import Path
import onnx
import onnxruntime as ort
p=argparse.ArgumentParser()
p.add_argument('source')
p.add_argument('output')
a=p.parse_args()
m=onnx.load(a.source)
for dim,size in zip(m.graph.input[0].type.tensor_type.shape.dim,[1,1,3,182,308]):
    dim.ClearField('dim_param')
    dim.dim_value=size
fixed=str(Path(a.output).with_suffix('.fixed.onnx'))
onnx.save(m,fixed)
opts=ort.SessionOptions()
opts.intra_op_num_threads=2
opts.inter_op_num_threads=1
opts.graph_optimization_level=ort.GraphOptimizationLevel.ORT_ENABLE_BASIC
opts.optimized_model_filepath=a.output
ort.InferenceSession(fixed,sess_options=opts,providers=['CPUExecutionProvider'])
m=onnx.load(a.output)
print('Frozen nodes:',len(m.graph.node))
assert all(n.domain in ('','ai.onnx') for n in m.graph.node), 'Nonportable fused operations'
onnx.checker.check_model(m)
