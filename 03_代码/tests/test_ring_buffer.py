import unittest

from ds.ring_buffer import RingBuffer


class RingBufferTest(unittest.TestCase):
    def test_fifo_overwrite(self):
        buf = RingBuffer[int](3)
        for value in range(5):
            buf.append(value)
        self.assertEqual(buf.snapshot(), [2, 3, 4])
        self.assertEqual(len(buf), 3)
        self.assertEqual(buf.dropped, 2)
        self.assertEqual(buf.total_appended, 5)
        self.assertTrue(buf.is_full)

    def test_latest_and_newest(self):
        buf = RingBuffer[str](4)
        for value in "abcd":
            buf.append(value)
        self.assertEqual(buf.latest(2), ["c", "d"])
        self.assertEqual(buf.newest(), "d")
        self.assertEqual(buf.oldest(), "a")
        self.assertEqual(buf.latest(10), ["a", "b", "c", "d"])
        self.assertEqual(buf.latest(0), [])

    def test_empty(self):
        buf = RingBuffer[int](2)
        self.assertIsNone(buf.newest())
        self.assertIsNone(buf.oldest())
        self.assertEqual(list(buf), [])
        self.assertFalse(buf)

    def test_clear_keeps_capacity(self):
        buf = RingBuffer[int](2)
        buf.append(1)
        buf.append(2)
        buf.clear()
        self.assertEqual(len(buf), 0)
        self.assertEqual(buf.capacity, 2)
        buf.append(9)
        self.assertEqual(buf.snapshot(), [9])

    def test_invalid_capacity(self):
        for bad in (0, -1, 1.5):
            with self.assertRaises(ValueError):
                RingBuffer(bad)

    def test_wrap_ordering(self):
        buf = RingBuffer[int](3)
        for value in [1, 2, 3, 4, 5]:
            buf.append(value)
        self.assertEqual(buf.latest(1), [5])
        self.assertEqual(buf.latest(3), [3, 4, 5])
        self.assertEqual(buf.oldest(), 3)


if __name__ == "__main__":
    unittest.main()
