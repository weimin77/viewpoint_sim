"""The flat RGB collector must produce stable names and never touch the source.

The script is exercised as a subprocess so the test also covers argument
parsing and the user-facing error paths.
"""
import csv
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT/'scripts/collect_rgb.py'


class CollectRgbTests(unittest.TestCase):
    def make_result(self, folder, ids=('h00_c0001', 'h01_c0002')):
        folder = Path(folder)
        views = []
        for index, view_id in enumerate(ids):
            view_folder = folder/view_id
            view_folder.mkdir(parents=True)
            for kind in ('rgb', 'food_mask'):
                Image.fromarray(np.full((4, 6, 3), index+1, np.uint8)).save(view_folder/(kind+'.png'))
            views.append(dict(id=view_id, height_index=index,
                              height_above_tray_m=0.2+0.04*index,
                              z_perturbation_m=0.0,
                              camera_height_above_tray_m=0.2+0.04*index))
        (folder/'views.json').write_text(json.dumps(views), encoding='utf-8')
        return folder

    def run_script(self, *args):
        return subprocess.run([sys.executable, str(SCRIPT), *map(str, args)],
                              capture_output=True, text=True)

    def test_collects_rgb_with_ordered_names_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self.make_result(Path(tmp)/'run')
            done = self.run_script(result, '--output', Path(tmp)/'flat')
            self.assertEqual(done.returncode, 0, done.stderr)
            flat = Path(tmp)/'flat'
            names = sorted(p.name for p in flat.iterdir())
            self.assertEqual(names, ['000_h00_c0001.png', '001_h01_c0002.png', 'manifest.csv'])
            with (flat/'manifest.csv').open(encoding='utf-8') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r['file'] for r in rows], ['000_h00_c0001.png', '001_h01_c0002.png'])
            self.assertEqual([r['folder'] for r in rows], ['h00_c0001', 'h01_c0002'])
            # The source result directory keeps only its own view folders.
            self.assertEqual(sorted(p.name for p in result.iterdir()), ['h00_c0001', 'h01_c0002', 'views.json'])

    def test_extra_kinds_get_a_suffix(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self.make_result(Path(tmp)/'run')
            done = self.run_script(result, '--output', Path(tmp)/'flat', '--kinds', 'rgb,food_mask')
            self.assertEqual(done.returncode, 0, done.stderr)
            names = sorted(p.name for p in (Path(tmp)/'flat').iterdir())
            self.assertIn('000_h00_c0001.png', names)
            self.assertIn('000_h00_c0001_food_mask.png', names)

    def test_refuses_non_empty_destination_unless_forced(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self.make_result(Path(tmp)/'run')
            flat = Path(tmp)/'flat'
            flat.mkdir()
            (flat/'stale.txt').write_text('x', encoding='utf-8')
            refused = self.run_script(result, '--output', flat)
            self.assertEqual(refused.returncode, 1)
            self.assertIn('not empty', refused.stderr)
            forced = self.run_script(result, '--output', flat, '--force')
            self.assertEqual(forced.returncode, 0, forced.stderr)

    def test_reports_missing_source_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            done = self.run_script(Path(tmp)/'nope', '--output', Path(tmp)/'flat')
            self.assertEqual(done.returncode, 1)
            self.assertIn('no views.json', done.stderr)

    def test_rejects_unknown_kind(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = self.make_result(Path(tmp)/'run')
            done = self.run_script(result, '--output', Path(tmp)/'flat', '--kinds', 'rgb,nope')
            self.assertEqual(done.returncode, 2)
            self.assertIn('Unknown image kind', done.stderr)


class CollectImagesApiTests(unittest.TestCase):
    """The shared function used by both `run` and the CLI."""

    def make_result(self, folder):
        folder = Path(folder)
        views = []
        for index, view_id in enumerate(('h00_c0001', 'h01_c0002')):
            view_folder = folder/view_id
            view_folder.mkdir(parents=True)
            Image.fromarray(np.full((4, 6, 3), index+1, np.uint8)).save(view_folder/'rgb.png')
            views.append(dict(id=view_id, height_index=index, height_above_tray_m=0.2,
                              z_perturbation_m=0.0, camera_height_above_tray_m=0.21))
        (folder/'views.json').write_text(json.dumps(views), encoding='utf-8')
        return folder, views

    def test_links_are_relative_and_source_is_untouched(self):
        from viewpoint_sim.acquisition import collect_images
        with tempfile.TemporaryDirectory() as tmp:
            result, views = self.make_result(Path(tmp)/'run')
            rows, manifest = collect_images(result, Path(tmp)/'flat', records=views,
                                            kinds=('rgb',), mode='link')
            self.assertEqual(len(rows), 2)
            link = Path(tmp)/'flat'/'000_h00_c0001.png'
            self.assertTrue(link.is_symlink())
            self.assertFalse(os.path.isabs(os.readlink(link)))
            self.assertTrue(link.resolve().is_file())
            self.assertTrue(manifest.is_file())
            self.assertEqual(sorted(p.name for p in result.iterdir()),
                             ['h00_c0001', 'h01_c0002', 'views.json'])

    def test_missing_view_image_is_reported(self):
        from viewpoint_sim.acquisition import collect_images
        with tempfile.TemporaryDirectory() as tmp:
            result, views = self.make_result(Path(tmp)/'run')
            (result/'h01_c0002'/'rgb.png').unlink()
            with self.assertRaises(FileNotFoundError):
                collect_images(result, Path(tmp)/'flat', records=views)

    def test_manifest_carries_height_metadata(self):
        from viewpoint_sim.acquisition import collect_images
        with tempfile.TemporaryDirectory() as tmp:
            result, views = self.make_result(Path(tmp)/'run')
            collect_images(result, Path(tmp)/'flat', records=views)
            with (Path(tmp)/'flat'/'manifest.csv').open(encoding='utf-8') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([r['id'] for r in rows], ['h00_c0001', 'h01_c0002'])
            self.assertEqual(rows[1]['height_above_tray_m'], '0.2')


if __name__ == '__main__':
    unittest.main()
