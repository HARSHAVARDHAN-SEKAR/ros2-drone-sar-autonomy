import numpy as np

from drone_exploration.grid_utils import astar, cluster_mask, frontier_mask, inflate_obstacles, nearest_free_cell


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


def test_nearest_free_cell_clamps_one_cell_map_boundary_drift():
    g = np.zeros((5, 5), dtype=np.int16)

    assert nearest_free_cell(g, (-1, 2)) == (0, 2)
    assert nearest_free_cell(g, (5, 2)) == (4, 2)
    assert nearest_free_cell(g, (2, -1)) == (2, 0)
    assert nearest_free_cell(g, (2, 5)) == (2, 4)

    # A vehicle genuinely far outside the current map is still rejected.
    assert nearest_free_cell(g, (-2, 2)) is None
    assert nearest_free_cell(g, (7, 2)) is None


def test_frontier_cluster_uses_diagonal_connectivity():
    mask = np.zeros((5, 5), dtype=bool)
    mask[1, 1] = True
    mask[2, 2] = True
    mask[3, 3] = True

    clusters = cluster_mask(mask, min_size=3)

    assert len(clusters) == 1
    assert len(clusters[0]) == 3
