"""Bounded DA3 inference on Jetson; outputs are relative, uncalibrated geometry."""
import argparse
import ctypes as C
import json
from pathlib import Path
import resource
import time
import cv2
import numpy as np


def preprocess(path, height, width):
    bgr = cv2.imread(str(path))
    if bgr is None:
        raise ValueError('Cannot decode %s' % path)
    return preprocess_bgr(bgr, height, width)


def preprocess_bgr(bgr, height=182, width=308):
    # Preserve aspect ratio; crop centrally to the fixed patch-grid shape.
    scale = max(width / bgr.shape[1], height / bgr.shape[0])
    resized = cv2.resize(bgr, (max(width, round(bgr.shape[1]*scale)), max(height, round(bgr.shape[0]*scale))), interpolation=cv2.INTER_CUBIC)
    y, x = (resized.shape[0]-height)//2, (resized.shape[1]-width)//2
    rgb = cv2.cvtColor(resized[y:y+height, x:x+width], cv2.COLOR_BGR2RGB)
    x = rgb.astype(np.float32) / 255.0
    x = (x - np.array([.485, .456, .406], np.float32)) / np.array([.229, .224, .225], np.float32)
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None, None]), rgb


class TrtEngine:
    def __init__(self, path):
        import tensorrt as trt
        self.trt = trt
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if self.engine is None:
            raise RuntimeError('Cannot deserialize engine')
        self.context = self.engine.create_execution_context()
        self.cuda = C.CDLL('/usr/local/cuda/lib64/libcudart.so')
        self.cuda.cudaMalloc.argtypes = [C.POINTER(C.c_void_p), C.c_size_t]
        self.cuda.cudaFree.argtypes = [C.c_void_p]
        self.cuda.cudaMemcpy.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.c_int]
        self.buffers = []
        for i in range(self.engine.num_bindings):
            shape = tuple(self.context.get_binding_shape(i))
            if min(shape) <= 0:
                raise ValueError('Build a fixed-shape engine first')
            dtype = {trt.float32: np.float32, trt.float16: np.float16, trt.int32: np.int32}[self.engine.get_binding_dtype(i)]
            host = np.empty(shape, dtype=dtype)
            device = C.c_void_p()
            self.check(self.cuda.cudaMalloc(C.byref(device), host.nbytes))
            self.buffers.append((self.engine.get_binding_name(i), self.engine.binding_is_input(i), host, device))

    @staticmethod
    def check(code):
        if code != 0: raise RuntimeError('CUDA error %s' % code)

    def run(self, x):
        for name, is_input, host, device in self.buffers:
            if is_input:
                if x.shape != host.shape: raise ValueError((x.shape, host.shape))
                np.copyto(host, x)
                self.check(self.cuda.cudaMemcpy(device, C.c_void_p(host.ctypes.data), host.nbytes, 1))
        if not self.context.execute_v2([d.value for _, _, _, d in self.buffers]):
            raise RuntimeError('TensorRT execution failed')
        out = {}
        for name, is_input, host, device in self.buffers:
            if not is_input:
                self.check(self.cuda.cudaMemcpy(C.c_void_p(host.ctypes.data), device, host.nbytes, 2))
                out[name] = host.copy()
        return out

    def close(self):
        for _, _, _, device in self.buffers:
            self.check(self.cuda.cudaFree(device))
        self.buffers = []


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--model', required=True)
    p.add_argument('--backend', choices=['ort', 'trt'], required=True)
    p.add_argument('--images', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--count', type=int, default=3)
    p.add_argument('--height', type=int, default=182)
    p.add_argument('--width', type=int, default=308)
    p.add_argument('--compare', help='Directory of reference NPZ outputs for identical inputs')
    a = p.parse_args()
    if min(a.count, a.height, a.width) <= 0 or a.height % 14 or a.width % 14:
        p.error('Positive count and image dimensions divisible by 14 are required')
    output = Path(a.output)
    output.mkdir(parents=True, exist_ok=False)
    paths = sorted(Path(a.images).glob('*.jpg'))
    if not paths: raise ValueError('No JPEG inputs')
    paths = [paths[i] for i in np.linspace(0, len(paths)-1, min(a.count, len(paths))).astype(int)]
    start = time.perf_counter()
    if a.backend == 'ort':
        import onnxruntime as ort
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        session = ort.InferenceSession(a.model, sess_options=opts, providers=['CPUExecutionProvider'])
        names = [v.name for v in session.get_outputs()]
        run = lambda x: dict(zip(names, session.run(None, {'pixel_values': x})))
        version = ort.__version__
    else:
        session = TrtEngine(a.model)
        run = session.run
        version = session.trt.__version__
    report = {'backend': a.backend, 'version': version, 'model': str(Path(a.model).resolve()),
              'units': 'relative; not metres', 'calibrated': False, 'fusion_enabled': False,
              'input_shape': [1, 1, 3, a.height, a.width], 'load_seconds': time.perf_counter()-start, 'frames': []}
    for i, path in enumerate(paths):
        x, rgb = preprocess(path, a.height, a.width)
        start = time.perf_counter()
        pred = run(x)
        elapsed = time.perf_counter()-start
        for name, value in pred.items():
            if not np.isfinite(value).all(): raise ValueError('Non-finite output: '+name)
        depth = pred['predicted_depth'].squeeze()
        if depth.shape != (a.height, a.width) or not (depth > 0).all():
            raise ValueError('Invalid depth output')
        np.savez_compressed(output / (path.stem+'.npz'), **pred)
        lo, hi = np.percentile(depth, [2, 98])
        colored = cv2.applyColorMap(np.clip((depth-lo)/max(hi-lo, 1e-6)*255, 0, 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)
        panel = np.concatenate([cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), colored], axis=1)
        cv2.putText(panel, 'RGB | DA3 relative depth (not metres)', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, .35, (255,255,255), 1)
        cv2.imwrite(str(output / (path.stem+'.jpg')), panel)
        row = {'source': str(path.resolve()), 'seconds': elapsed, 'depth_min': float(depth.min()), 'depth_max': float(depth.max())}
        if a.compare:
            reference = np.load(Path(a.compare)/(path.stem+'.npz'))
            ref = reference['predicted_depth'].squeeze()
            row['reference_relative_mae'] = float(np.mean(np.abs(depth-ref)/np.maximum(np.abs(ref),1e-6)))
            row['reference_max_abs_error'] = float(np.max(np.abs(depth-ref)))
            row['output_relative_l2'] = {name: float(np.linalg.norm(value-reference[name])/max(np.linalg.norm(reference[name]), 1e-6)) for name,value in pred.items()}
            reference.close()
        report['frames'].append(row)
        print(json.dumps(row), flush=True)
    report['peak_rss_mib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024
    if a.backend == 'trt': session.close()
    (output/'benchmark.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
