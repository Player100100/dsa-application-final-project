import unittest

from ds.cluster_tracker import ClusterTracker


def blob(x, y, n=3):
    return [(x + i * 0.01, y) for i in range(n)]


class ClusterTrackerTest(unittest.TestCase):
    def test_static_cluster_not_dynamic(self):
        tracker = ClusterTracker(dynamic_speed=0.15)
        points = blob(1.0, 0.0)
        tracker.update(points, [[0, 1, 2]], now=0.0)
        points = blob(1.0, 0.0)
        tracks = tracker.update(points, [[0, 1, 2]], now=0.5)
        self.assertEqual(len(tracks), 1)
        self.assertFalse(tracks[0].dynamic)
        self.assertLess(tracks[0].speed, 0.15)

    def test_moving_cluster_becomes_dynamic(self):
        # smoothing=1.0：直接采用本帧速度，便于断言 0.3 m / 0.5 s = 0.6 m/s
        tracker = ClusterTracker(dynamic_speed=0.15, smoothing=1.0)
        tracker.update(blob(1.0, 0.0), [[0, 1, 2]], now=0.0)
        tracks = tracker.update(blob(1.3, 0.0), [[0, 1, 2]], now=0.5)
        self.assertEqual(len(tracks), 1)
        self.assertTrue(tracks[0].dynamic)
        self.assertAlmostEqual(tracks[0].speed, 0.6, places=1)

    def test_smoothing_damps_velocity(self):
        tracker = ClusterTracker(dynamic_speed=0.15, smoothing=0.5)
        tracker.update(blob(1.0, 0.0), [[0, 1, 2]], now=0.0)
        tracks = tracker.update(blob(1.3, 0.0), [[0, 1, 2]], now=0.5)
        self.assertAlmostEqual(tracks[0].speed, 0.3, places=1)

    def test_new_track_when_outside_gate(self):
        tracker = ClusterTracker(gate=0.3)
        tracker.update(blob(1.0, 0.0), [[0, 1, 2]], now=0.0)
        tracks = tracker.update(blob(2.0, 0.0), [[0, 1, 2]], now=0.5)
        self.assertEqual(len(tracks), 1)
        self.assertEqual(len(tracker.tracks), 2)

    def test_old_track_expires(self):
        tracker = ClusterTracker(max_age=1.0)
        tracker.update(blob(1.0, 0.0), [[0, 1, 2]], now=0.0)
        tracker.update([], [], now=2.0)
        self.assertEqual(tracker.tracks, [])

    def test_dynamic_in_front(self):
        tracker = ClusterTracker(dynamic_speed=0.15)
        tracker.update(blob(0.6, 0.0), [[0, 1, 2]], now=0.0)
        tracker.update(blob(0.9, 0.0), [[0, 1, 2]], now=0.5)
        track = tracker.dynamic_in_front(now=0.5, max_distance=1.0)
        self.assertIsNotNone(track)
        self.assertGreater(track.centroid[0], 0.0)

    def test_dynamic_behind_not_in_front(self):
        tracker = ClusterTracker(dynamic_speed=0.15)
        tracker.update(blob(-0.6, 0.0), [[0, 1, 2]], now=0.0)
        tracker.update(blob(-0.9, 0.0), [[0, 1, 2]], now=0.5)
        self.assertIsNone(tracker.dynamic_in_front(now=0.5, max_distance=1.0))


if __name__ == "__main__":
    unittest.main()
