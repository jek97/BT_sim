from amiga_ros2_planners.ploughing import bresenham_cells, cell_index


def test_cell_index_rounds_to_nearest():
    assert cell_index(5.4, 1.0) == 5
    assert cell_index(5.6, 1.0) == 6
    assert cell_index(-0.6, 1.0) == -1


def test_cell_index_respects_cell_size():
    assert cell_index(2.4, 2.0) == 1
    assert cell_index(2.6, 2.0) == 1
    assert cell_index(3.1, 2.0) == 2


def test_bresenham_cells_same_point():
    assert bresenham_cells(2, 2, 2, 2) == [(2, 2)]


def test_bresenham_cells_includes_both_endpoints():
    cells = bresenham_cells(0, 0, 3, 1)
    assert cells[0] == (0, 0)
    assert cells[-1] == (3, 1)


def test_bresenham_cells_horizontal_line():
    assert bresenham_cells(0, 0, 4, 0) == [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]


def test_bresenham_cells_vertical_line():
    assert bresenham_cells(0, 0, 0, 3) == [(0, 0), (0, 1), (0, 2), (0, 3)]


def test_bresenham_cells_symmetric_regardless_of_direction():
    forward = bresenham_cells(0, 0, 5, 3)
    backward = bresenham_cells(5, 3, 0, 0)
    assert forward == list(reversed(backward))
