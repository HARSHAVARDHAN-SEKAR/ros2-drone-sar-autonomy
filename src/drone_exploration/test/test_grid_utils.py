import numpy as np

from drone_exploration.grid_utils import astar, cluster_mask, frontier_mask, inflate_obstacles


def test_frontier_is_free_cell_adjacent_to_unknown():
    g = np.full((5, 5), -1, dtype=np.int16)
    g[1:4, 1:4] = 0
    mask = frontier_mask(g)
    assert mask[1, 2]
    assert not mask[2, 2]


def test_inflation_and_astar_avoid_obstacle():
    g = np.zeros((9, 9), dtype=np.int16)
    g[4, 4] = 100
    blocked = inflate_obstacles(g, 1)
    assert blocked[4, 4]
    assert blocked[4, 5]
    path = astar(g, (1, 4), (7, 4), blocked)
    assert path
    assert (4, 4) not in path
    assert (5, 4) not in path


def test_astar_rejects_unknown_shortcut():
    g = np.zeros((5, 7), dtype=np.int16)
    g[2, 2:5] = -1
    path = astar(g, (0, 2), (6, 2))
    assert path
    assert all(g[y, x] >= 0 for x, y in path)
