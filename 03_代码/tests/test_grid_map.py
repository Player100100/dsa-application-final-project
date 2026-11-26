import tempfile
import unittest
import warnings
from pathlib import Path

from ds.grid_map import FREE, OCCUPIED, UNKNOWN, GridMap


class GridMapTest(unittest.TestCase):
    def test_default_unknown(self):
        grid = GridMap(4, 3, 0.1)
        self.assertEqual(len(grid.data), 12)
        self.assertEqual(grid.count(UNKNOWN), 12)

    def test_set_get_and_bounds(self):
        grid = GridMap(4, 3, 0.1)
        grid.set(1, 2, OCCUPIED)
        self.assertEqual(grid.get(1, 2), OCCUPIED)
        with self.assertRaises(IndexError):
            grid.get(4, 0)
        with self.assertRaises(IndexError):
            grid.set(-1, 0, 0)
        with self.assertRaises(ValueError):
            grid.set(0, 0, 101)

    def test_world_roundtrip(self):
        grid = GridMap(10, 10, 0.1, origin=(-1.0, -2.0))
        x, y = grid.world_to_cell(-0.95, -1.95)
        self.assertEqual((x, y), (0, 0))
        wx, wy = grid.cell_to_world(x, y)
        self.assertAlmostEqual(wx, -0.95)
        self.assertAlmostEqual(wy, -1.95)
        # 原点 (-1,-2)、0.1 m/格：世界 (0,0) 在第 10 列、第 20 行（可越界，换算本身合法）
        self.assertEqual(grid.world_to_cell(0.0, 0.0), (10, 20))

    def test_inflate(self):
        grid = GridMap(20, 20, 0.05)
        grid.fill(FREE)
        grid.set(10, 10, OCCUPIED)
        blocked = grid.inflated_mask(inflate_m=0.1)  # 半径 2 格
        self.assertTrue(blocked[10 * 20 + 10])
        self.assertTrue(blocked[12 * 20 + 10])
        self.assertTrue(blocked[10 * 20 + 12])
        self.assertFalse(blocked[13 * 20 + 10])
        self.assertFalse(blocked[10 * 20 + 10 - 3])

    def test_save_load_trinary(self):
        with tempfile.TemporaryDirectory() as tmp:
            grid = GridMap(6, 4, 0.1, origin=(-0.3, 0.2))
            grid.fill(FREE)
            grid.set(0, 0, UNKNOWN)
            grid.set(1, 1, OCCUPIED)
            grid.set(2, 2, 40)   # 中间值落盘后应变未知
            grid.set(3, 3, 10)   # 空闲
            path = grid.save(Path(tmp) / "map.yaml")
            self.assertTrue((Path(tmp) / "map.pgm").exists())
            text = path.read_text(encoding="utf-8")
            self.assertIn("mode: trinary", text)
            self.assertIn("resolution: 0.1", text)

            loaded = GridMap.load(path)
            self.assertEqual((loaded.width, loaded.height), (6, 4))
            self.assertAlmostEqual(loaded.resolution, 0.1)
            self.assertAlmostEqual(loaded.origin_x, -0.3)
            self.assertAlmostEqual(loaded.origin_y, 0.2)
            self.assertEqual(loaded.get(0, 0), UNKNOWN)
            self.assertEqual(loaded.get(1, 1), OCCUPIED)
            self.assertEqual(loaded.get(2, 2), UNKNOWN)
            self.assertEqual(loaded.get(3, 3), FREE)
            self.assertEqual(loaded.get(5, 0), FREE)

    def test_free_thresh_warning_and_override(self):
        """外部地图 free_thresh > 0.196 时未知会变空闲；显式覆盖可修正。"""
        with tempfile.TemporaryDirectory() as tmp:
            grid = GridMap(2, 2, 0.1)
            grid.fill(UNKNOWN)
            path = grid.save(Path(tmp) / "m.yaml")
            path.write_text(
                path.read_text(encoding="utf-8").replace(
                    "free_thresh: 0.196", "free_thresh: 0.25"
                ),
                encoding="utf-8",
            )
            with self.assertWarns(RuntimeWarning):
                loose = GridMap.load(path)
            self.assertEqual(loose.get(0, 0), FREE)

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                strict = GridMap.load(path, free_thresh=0.196)
            self.assertEqual(strict.get(0, 0), UNKNOWN)

    def test_save_flips_rows(self):
        """地图第 0 行（y 最小）应写在 PGM 的最后一行。"""
        with tempfile.TemporaryDirectory() as tmp:
            grid = GridMap(2, 2, 1.0)
            grid.fill(FREE)
            grid.set(0, 0, OCCUPIED)  # 左下角
            path = grid.save(Path(tmp) / "m.yaml")
            pgm = (Path(tmp) / "m.pgm").read_bytes()
            header_len = pgm.index(b"255\n") + 4
            pixels = pgm[header_len:]
            self.assertEqual(pixels[0], 254)   # 图像左上：地图 (0,1) 空闲
            self.assertEqual(pixels[1], 254)
            self.assertEqual(pixels[2], 0)     # 图像左下：地图 (0,0) 占据
            self.assertEqual(pixels[3], 254)


if __name__ == "__main__":
    unittest.main()
