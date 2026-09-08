"""No ROS graph: map bundle validation and no-clobber publication."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'go2_nav2'))
from go2_nav2.map_bundle import publish_bundle, save_map, validate_map


class MapBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def bundle(self, directory, name='room'):
        directory.mkdir(exist_ok=True)
        stem = directory / name
        for suffix in ('.pgm', '.posegraph', '.data'):
            stem.with_suffix(suffix).write_bytes(b'fixture')
        stem.with_suffix('.yaml').write_text(yaml.safe_dump({
            'image': name+'.pgm', 'resolution': .05, 'origin': [0., 0., 0.]}))
        return stem

    def test_complete_bundle_is_accepted(self):
        stem = self.bundle(self.root)
        self.assertEqual(validate_map(stem.with_suffix('.yaml')), stem.with_suffix('.yaml'))

    def test_each_missing_component_blocks_localization(self):
        for suffix in ('.pgm', '.posegraph', '.data'):
            with self.subTest(suffix=suffix):
                stem = self.bundle(self.root)
                stem.with_suffix(suffix).unlink()
                with self.assertRaises(ValueError):
                    validate_map(stem.with_suffix('.yaml'))

    def test_invalid_resolution_is_rejected(self):
        stem = self.bundle(self.root)
        path = stem.with_suffix('.yaml')
        data = yaml.safe_load(path.read_text())
        data['resolution'] = float('nan')
        path.write_text(yaml.safe_dump(data))
        with self.assertRaises(ValueError):
            validate_map(path)

    def test_save_refuses_existing_file_before_ros(self):
        (self.root/'room.data').write_bytes(b'old-map')
        with patch('go2_nav2.map_bundle.subprocess.run') as run:
            with self.assertRaises(ValueError):
                save_map('room', self.root)
            run.assert_not_called()
        self.assertEqual((self.root/'room.data').read_bytes(), b'old-map')

    def test_save_rejects_path_traversal(self):
        with self.assertRaises(ValueError):
            save_map('../room', self.root)

    def test_publish_does_not_overwrite_and_rolls_back_only_new_links(self):
        staged = self.bundle(self.root/'stage')
        (self.root/'room.posegraph').write_bytes(b'user-data')
        with self.assertRaises(FileExistsError):
            publish_bundle(staged, self.root/'room')
        self.assertFalse((self.root/'room.pgm').exists())
        self.assertFalse((self.root/'room.yaml').exists())
        self.assertEqual((self.root/'room.posegraph').read_bytes(), b'user-data')

    def test_complete_save_publishes_portable_bundle(self):
        def ros(args, **kwargs):
            if '-f' in args:
                staged = Path(args[args.index('-f')+1])
                self.bundle(staged.parent, staged.name)
        with patch('go2_nav2.map_bundle.subprocess.run', side_effect=ros):
            save_map('room', self.root)
        self.assertEqual(validate_map(self.root/'room.yaml'), self.root/'room.yaml')
        self.assertEqual(yaml.safe_load((self.root/'room.yaml').read_text())['image'], 'room.pgm')


if __name__ == '__main__':
    unittest.main()
