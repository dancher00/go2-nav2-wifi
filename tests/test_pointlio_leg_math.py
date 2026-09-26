"""Numerical differentiation of the actual C++ foot observation implementation."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class LegObservationMathTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++') and Path('/usr/include/eigen3/Eigen/Core').exists(),
                         'C++/Eigen development headers required')
    def test_fk_and_filter_jacobians(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(Path(directory) / 'leg-math')
            subprocess.run(['g++', '-std=c++14', '-I/usr/include/eigen3',
                            '-I' + str(root / 'ws/scripts/pointlio-leg'),
                            str(root / 'tests/pointlio_leg_math.cpp'), '-o', binary],
                           check=True, capture_output=True, text=True)
            subprocess.run([binary], check=True, capture_output=True, text=True)
