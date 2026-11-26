import math
import unittest

from ds.astar import astar, clear_around, dijkstra, path_to_world, smooth_path
from ds.grid_map import FREE, OCCUPIED, UNKNOWN, GridMap


def open_grid(size=6):
    grid = GridMap(size, size, 0.1)
    grid.fill(FREE)
    return grid


class AStarTest(unittest.TestCase):
    def test_straight_line(self):
        grid = open_grid()
        result = astar(grid, (0, 0), (5, 0))
        self.assertTrue(result.found)
        self.assertEqual(result.path[0], (0, 0))
        self.assertEqual(result.path[-1], (5, 0))
        self.assertAlmostEqual(result.cost, 5.0)

    def test_goal_blocked(self):
        grid = open_grid()
        grid.set(5, 0, OCCUPIED)
        result = astar(grid, (0, 0), (5, 0))
        self.assertFalse(result.found)
        self.assertEqual(result.path, [])

    def test_out_of_bounds(self):
        grid = open_grid()
        self.assertFalse(astar(grid, (0, 0), (99, 0)).found)
        self.assertFalse(astar(grid, (-1, 0), (0, 0)).found)

    def test_wall_with_gap(self):
        grid = open_grid(7)
        for y in range(7):
            grid.set(3, y, OCCUPIED)
        grid.set(3, 3, FREE)  # 中间留门
        result = astar(grid, (0, 0), (6, 0))
        self.assertTrue(result.found)
        self.assertIn((3, 3), result.path)

    def test_no_path(self):
        grid = open_grid(7)
        for y in range(7):
            grid.set(3, y, OCCUPIED)
        result = astar(grid, (0, 0), (6, 0))
        self.assertFalse(result.found)
        self.assertIn("没有可行路径", result.reason)

    def test_no_corner_cutting(self):
        grid = open_grid(6)
        grid.set(2, 1, OCCUPIED)
        grid.set(1, 2, OCCUPIED)
        result = astar(grid, (1, 1), (2, 2))
        self.assertTrue(result.found)
        # 不能斜穿 (1,1)->(2,2) 的夹角；绕行代价必然大于 sqrt(2)
        self.assertGreater(result.cost, 1.5)

    def test_diagonal_checked(self):
        grid = open_grid(3)
        result = astar(grid, (0, 0), (2, 2))
        self.assertTrue(result.found)
        self.assertEqual(len(result.path), 3)
        self.assertAlmostEqual(result.cost, 2 * math.sqrt(2))
        world = path_to_world(grid, result.path)
        self.assertEqual(len(world), 3)
        self.assertAlmostEqual(world[0][0], 0.05)
        self.assertAlmostEqual(world[0][1], 0.05)

    def test_astar_equals_dijkstra_cost(self):
        grid = open_grid(30)
        for y in range(1, 29, 3):
            for x in range(1, 29):
                if x != 15:
                    grid.set(x, y, OCCUPIED)
        a = astar(grid, (2, 2), (27, 27))
        d = dijkstra(grid, (2, 2), (27, 27))
        self.assertTrue(a.found and d.found)
        self.assertAlmostEqual(a.cost, d.cost)
        # 一致的启发式下，A* 展开节点不应多于 Dijkstra
        self.assertLessEqual(a.expanded, d.expanded)

    def test_unknown_blocking(self):
        grid = GridMap(5, 5, 0.1)
        grid.fill(FREE)
        for y in range(5):
            grid.set(2, y, UNKNOWN)
        optimistic = astar(grid, (0, 2), (4, 2))
        cautious = astar(grid, (0, 2), (4, 2), unknown_blocked=True)
        self.assertTrue(optimistic.found)
        self.assertFalse(cautious.found)

    def test_inflation(self):
        grid = open_grid(10)
        grid.set(5, 5, OCCUPIED)
        blocked = grid.inflated_mask(inflate_m=0.3)  # 6 格
        result = astar(grid, (0, 0), (9, 9), blocked=blocked)
        self.assertTrue(result.found)
        for x, y in result.path:
            self.assertGreater(math.hypot(x - 5, y - 5) * 0.1, 0.29)

    def test_clear_around_unblocks_neighborhood(self):
        width = height = 5
        blocked = [True] * (width * height)
        cleared = clear_around(blocked, width, height, (2, 2), radius=1)
        for y in range(1, 4):
            for x in range(1, 4):
                self.assertFalse(cleared[y * width + x], (x, y))
        self.assertTrue(cleared[0])
        self.assertTrue(blocked[2 * width + 2], "不得修改原表")

    def test_clear_around_clips_at_border(self):
        width = height = 3
        blocked = [True] * 9
        cleared = clear_around(blocked, width, height, (0, 0), radius=2)
        self.assertFalse(any(cleared))

    def test_smooth_path(self):
        points = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
        smoothed = smooth_path(points)
        self.assertEqual(smoothed[0], (0, 0))
        self.assertEqual(smoothed[-1], (2, 2))
        self.assertLess(len(smoothed), len(points))


if __name__ == "__main__":
    unittest.main()
