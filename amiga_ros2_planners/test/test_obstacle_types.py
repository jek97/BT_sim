from amiga_ros2_planners.obstacle_types import Obstacle, resolve_obstacle_id

OBSTACLES = [
    Obstacle("tree_1", 0.0, 0.0, 0.5),
    Obstacle("tree_5", 10.0, 0.0, 0.5),
    Obstacle("tree_12", 20.0, 0.0, 0.5),
]


def test_exact_match():
    assert resolve_obstacle_id(OBSTACLES, "tree_5").id == "tree_5"


def test_bare_numeric_id_resolves_to_tree():
    assert resolve_obstacle_id(OBSTACLES, "5").id == "tree_5"


def test_problog_style_obstacle_id_resolves_by_embedded_number():
    assert resolve_obstacle_id(OBSTACLES, "obs5").id == "tree_5"


def test_multi_digit_number_resolves_correctly():
    assert resolve_obstacle_id(OBSTACLES, "obs12").id == "tree_12"


def test_unknown_id_returns_none():
    assert resolve_obstacle_id(OBSTACLES, "obs999") is None
    assert resolve_obstacle_id(OBSTACLES, "no_digits_here") is None
