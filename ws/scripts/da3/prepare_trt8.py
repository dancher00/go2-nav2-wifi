"""Lower DA3 ONNX LayerNorm for TensorRT 8.5; retain original model separately."""
import argparse
from pathlib import Path
import numpy as np
import onnx
from onnx import helper as h, numpy_helper as nh

p = argparse.ArgumentParser()
p.add_argument('source')
p.add_argument('output')
a = p.parse_args()
m = onnx.load(a.source)
constants = {t.name: nh.to_array(t) for t in m.graph.initializer}
nodes = []
count = 0
for n in m.graph.node:
    if n.op_type == 'LayerNormalization':
        attrs = {x.name: h.get_attribute_value(x) for x in n.attribute}
        if attrs.get('axis', -1) != -1 or len(n.output) != 1:
            raise ValueError('Only last-axis LayerNorm with one output is supported')
        prefix = 'trt8_ln_%d_' % count
        count += 1
        eps = prefix + 'eps'
        m.graph.initializer.append(nh.from_array(np.array(attrs.get('epsilon', 1e-5), np.float32), eps))
        def op(kind, ins, out, **kw):
            nodes.append(h.make_node(kind, ins, [prefix + out], name=prefix + out, **kw))
        op('ReduceMean', [n.input[0]], 'mean', axes=[-1], keepdims=1)
        op('Sub', [n.input[0], prefix+'mean'], 'center')
        op('Mul', [prefix+'center', prefix+'center'], 'square')
        op('ReduceMean', [prefix+'square'], 'var', axes=[-1], keepdims=1)
        op('Add', [prefix+'var', eps], 'epsvar')
        op('Sqrt', [prefix+'epsvar'], 'std')
        op('Div', [prefix+'center', prefix+'std'], 'norm')
        op('Mul', [prefix+'norm', n.input[1]], 'scaled')
        nodes.append(h.make_node('Add', [prefix+'scaled', n.input[2]], list(n.output), name=prefix+'bias'))
    else:
        if n.op_type in ('ReduceMax', 'ReduceMin', 'ReduceMean') and len(n.input) > 1:
            axes = constants[n.input[1]].tolist()
            del n.input[1:]
            n.attribute.append(h.make_attribute('axes', axes))
        if n.op_type == 'Split':
            for attr in list(n.attribute):
                if attr.name == 'num_outputs':
                    n.attribute.remove(attr)
        nodes.append(n)
del m.graph.node[:]
m.graph.node.extend(nodes)
for imp in m.opset_import:
    if imp.domain == '': imp.version = 17
onnx.checker.check_model(m)
Path(a.output).parent.mkdir(parents=True, exist_ok=True)
onnx.save(m, a.output)
print('Lowered LayerNorm nodes:', count)
