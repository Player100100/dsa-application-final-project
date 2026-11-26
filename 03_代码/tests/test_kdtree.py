import math
import random
import unittest

from ds.kdtree import KDTree, brute_force_knn


class KDTreeTest(unittest.TestCase):
    def setUp(self):
        rng = random.Random(2026)
        self.points = [(rng.uniform(-5, 5), rng.uniform(-5, 5)) for _ in range(300)]
        self.tree = KDTree(self.points)
        self.targets = [(rng.uniform(-5, 5), rng.uniform(-5, 5)) for _ in range(30)]

    def test_knn_matches_brute_force(self):
        for target in self.targets:
            got = [index for _, index in self.tree.knn(target, 5)]
            expected = [index for _, index in brute_force_knn(self.points, target, 5)]
            self.assertEqual(got, expected)

    def test_nearest(self):
        for target in self.targets[:10]:
            expected = brute_force_knn(self.points, target, 1)[0][1]
            self.assertEqual(self.tree.nearest(target), expected)

    def test_range_query(self):
        for target in self.targets[:10]:
            got = sorted(self.tree.range_query(target, 0.7))
            expected = sorted(
                i for i, p in enumerate(self.points)
                if math.dist(p, target) <= 0.7
            )
            self.assertEqual(got, expected)

    def test_range_query_negative(self):
        self.assertEqual(self.tree.range_query((0, 0), -1), [])

    def test_empty_tree(self):
        tree = KDTree([])
        self.assertIsNone(tree.nearest((0, 0)))
        self.assertEqual(tree.knn((0, 0), 3), [])
        self.assertEqual(tree.radius_clusters(1.0), [])

    def test_clusters_two_blobs(self):
        points = [(0.0, 0.0), (0.1, 0.0), (0.0, 0.1), (5.0, 5.0), (5.1, 5.0)]
        tree = KDTree(points)
        clusters = tree.radius_clusters(0.3, min_points=1)
        self.assertEqual(len(clusters), 2)
        self.assertEqual(sorted(clusters[0]), [0, 1, 2])
        self.assertEqual(sorted(clusters[1]), [3, 4])

    def test_cluster_min_points(self):
        points = [(0.0, 0.0), (0.1, 0.0), (9.0, 9.0)]
        clusters = KDTree(points).radius_clusters(0.2, min_points=2)
        self.assertEqual(clusters, [[0, 1]])


if __name__ == "__main__":
    unittest.main()
