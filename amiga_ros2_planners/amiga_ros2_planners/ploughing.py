"""
ploughing.py

Pure-Python primitives shared by move_to_node.py (which MARKS cells
ploughed) and condition_service_node.py (which QUERIES them for
PloughedAt/PloughedBetween) -- byte-for-byte ports of
basic_action_theory.pl's own cell_index/3 and bresenham_cells/5 (see
that file's own "ploughed/3" section for the full rationale: a plow
that is both hitched and deployed marks the macro-cell it passes
through as ploughed, discretized coarser than position itself so a
slightly wobbly pass still counts).

No ROS import here on purpose -- unit-testable standalone, same
"pure-Python core, ROS nodes call in" shape as planning_core.py/
polygon_geometry.py.
"""


def cell_index(value, cell_size):
    """round(value/cell_size) -- byte-for-byte port of
    basic_action_theory.pl's own cell_index/3."""
    return round(value / cell_size)


def bresenham_cells(cx0, cy0, cx1, cy1):
    """[(cx,cy), ...] -- every integer grid cell the standard Bresenham
    line algorithm visits walking from (cx0,cy0) to (cx1,cy1), INCLUDING
    both endpoints -- byte-for-byte port of basic_action_theory.pl's own
    bresenham_cells/5 (bresenham_sign/bresenham_x_step/bresenham_y_step/
    bresenham_walk folded into one iterative loop here rather than
    Prolog's own recursive/backtracking style)."""
    dx = abs(cx1 - cx0)
    dy = -abs(cy1 - cy0)
    sx = 1 if cx0 < cx1 else -1
    sy = 1 if cy0 < cy1 else -1
    err = dx + dy

    cells = []
    cx, cy = cx0, cy0
    while True:
        cells.append((cx, cy))
        if cx == cx1 and cy == cy1:
            return cells
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            cx += sx
        if e2 <= dx:
            err += dx
            cy += sy
