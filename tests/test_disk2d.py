import numpy as np

from drrt.checkers.disk2d import DiskPairChecker, DiskSpace, segment_rect_distance


def _sampled_rect_distance(a, b, rect, steps=2001):
    t = np.linspace(0, 1, steps)[:, None]
    p = np.asarray(a) * (1 - t) + np.asarray(b) * t
    dx = np.maximum.reduce([rect[0] - p[:, 0], p[:, 0] - rect[2], np.zeros(steps)])
    dy = np.maximum.reduce([rect[1] - p[:, 1], p[:, 1] - rect[3], np.zeros(steps)])
    return float(np.hypot(dx, dy).min())


def test_segment_rect_distance_matches_dense_sampling():
    rng = np.random.default_rng(0)
    rect = (3.0, 3.0, 6.0, 5.0)
    for _ in range(500):
        a, b = rng.uniform(0, 10, (2, 2))
        exact = segment_rect_distance(a, b, rect)
        assert abs(exact - _sampled_rect_distance(a, b, rect)) < 0.02


def test_pair_checker_matches_dense_sampling():
    rng = np.random.default_rng(1)
    checker = DiskPairChecker(radius=0.2)
    t = np.linspace(0, 1, 4001)[:, None]
    agree = 0
    total = 0
    for _ in range(1000):
        a0, a1, b0, b1 = rng.uniform(0, 3, (4, 2))
        separation = np.linalg.norm((a0 * (1 - t) + a1 * t) - (b0 * (1 - t) + b1 * t), axis=1).min()
        if abs(separation - 0.4) < 1e-3:
            continue  # sát biên, lấy mẫu không phân biệt được
        expected = separation >= 0.4
        got = checker.edge_free([tuple(a0), tuple(b0)], [tuple(a1), tuple(b1)])
        agree += got == expected
        total += 1
    assert agree == total


def test_crossing_disks_collide_even_when_endpoints_are_free():
    checker = DiskPairChecker(radius=0.2)
    assert not checker.edge_free([(0.0, 0.0), (2.0, 0.0)], [(2.0, 0.0), (0.0, 0.0)])
    assert checker.edge_free([(0.0, 0.0), (0.0, 2.0)], [(2.0, 0.0), (2.0, 2.0)])


def test_space_rejects_inflated_obstacle():
    space = DiskSpace(rects=[(4.0, 4.0, 6.0, 6.0)], clearance=0.2)
    assert not space.config_free((3.9, 5.0))
    assert space.config_free((3.7, 5.0))
    assert not space.edge_free((3.0, 5.0), (7.0, 5.0))


def test_default_space_treats_figure_rects_as_already_inflated():
    space = DiskSpace(rects=[(4.0, 4.0, 6.0, 6.0)])
    assert space.config_free((3.99, 5.0))
    assert not space.config_free((4.01, 5.0))
    assert not space.edge_free((3.0, 5.0), (7.0, 5.0))


def test_figure9_endpoints_are_free():
    space = DiskSpace()
    assert space.config_free((0.0, 0.0)) and space.config_free((9.0, 9.0))
