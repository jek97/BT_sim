#!/usr/bin/env python3
"""
export_orchard_map.py

Extracts the real orchard tree list from a checked-in
amiga_ros2_behavior_tree/examples/*.bin mission fixture (the same
length-prefixed two-frame TCP payload tcp_demux_node parses at
runtime -- see amiga_ros2_behavior_tree/README.md), converts it to
local (x,y) around a fixed datum, and writes out a standard ROS
map_server map.pgm/map.yaml pair -- the SAME format problog_project's
own planners.py loads via load_map_yaml, so BT_project can point
BT_PROBLEM_DIR/map.yaml straight at this output with no format
conversion of its own.

Deliberately ROS-free (plain Python + numpy/PIL/yaml only): this runs
standalone, with no ROS2 environment, no Gazebo, no running simulation
-- it reads a static fixture already checked into this repo and
rasterizes it exactly the way orchard_map_node.py would from a live
orchard topic (same planning_core.build_grid_map call), so the saved
map matches what the live simulation would extract at startup from the
same orchard.

Usage:
    python3 export_orchard_map.py \
        --bin ../../amiga_ros2_behavior_tree/examples/mv_10_60_sample.bin \
        --out ../maps

Two frames come out of the .bin file: an XML mission (frame 0, ignored
here) and the orchard JSON (frame 1, what this script actually uses).
See orchard_obstacles.py's own note on the two JSON shapes this repo's
fixtures carry ({"trees": [...]} vs. a bare [...] list) -- this script
accepts either.

Datum: by default, the FIRST tree's own (lat, lon) -- an arbitrary but
stable local origin, chosen so the output map is self-contained and
usable on its own (BT_project's own config.yaml start_x/start_y are
relative to whatever origin its map.yaml uses; they are NOT tied to
this simulation's own live datum_lat/datum_lon params -- see this
package's README's own note on why those two must match the ROBOT's
localization stack, which has nothing to do with this offline export).
Pass --datum-lat/--datum-lon explicitly to line this map up with a
different origin instead (e.g. this simulation's own live datum, if you
want the two to share a frame).
"""
import argparse
import json
import os
import struct
import sys

import numpy as np
import yaml
from PIL import Image

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_DIR = os.path.dirname(_THIS_DIR)
sys.path.insert(0, _PACKAGE_DIR)

from amiga_ros2_planners.gps import latlon_to_local  # noqa: E402
from amiga_ros2_planners.obstacle_types import Obstacle  # noqa: E402
from amiga_ros2_planners.planning_core import build_grid_map  # noqa: E402

# ROS map_server convention (see problog_project's own load_map_yaml):
# negate=0 means LOW pixel value ("dark") = occupied, HIGH ("light") = free.
PIXEL_FREE = 254
PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205


def read_frames(bin_path):
    """[bytes, ...] -- every 4-byte-big-endian-length-prefixed frame in
    a tcp_demux_node-style payload, same framing
    amiga_ros2_behavior_tree/README.md documents."""
    data = open(bin_path, "rb").read()
    frames = []
    offset = 0
    while offset < len(data):
        (length,) = struct.unpack_from(">I", data, offset)
        offset += 4
        frames.append(data[offset:offset + length])
        offset += length
    return frames


def load_trees_from_bin(bin_path):
    frames = read_frames(bin_path)
    if len(frames) < 2:
        raise ValueError(
            f"{bin_path}: expected 2 frames (XML, orchard JSON), got {len(frames)}")
    payload = json.loads(frames[1])
    trees = payload.get("trees", []) if isinstance(payload, dict) else payload
    return [t for t in trees if "lat" in t and "lon" in t]


def trees_to_obstacles(trees, datum_lat, datum_lon, tree_radius):
    obstacles = []
    for tree in trees:
        x, y = latlon_to_local(tree["lat"], tree["lon"], datum_lat, datum_lon)
        obstacles.append(Obstacle(f"tree_{tree['tree_index']}", x, y, tree_radius))
    return obstacles


def write_map(grid, out_dir, map_name):
    """OccupancyGridMap -> <out_dir>/<map_name>.pgm + .yaml, standard
    ROS map_server format."""
    # OccupancyGridMap row 0 = min y (nav_msgs/OccupancyGrid convention);
    # a PGM/map.yaml image's row 0 is the TOP of the picture = max y
    # (load_map_yaml flips vertically on load for exactly this reason)
    # -- flip once here so the file round-trips correctly either way.
    pixels = np.full(grid.data.shape, PIXEL_UNKNOWN, dtype=np.uint8)
    pixels[grid.data == 0] = PIXEL_FREE
    pixels[grid.data >= 100] = PIXEL_OCCUPIED
    pixels = np.flipud(pixels)

    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, f"{map_name}.pgm")
    yaml_path = os.path.join(out_dir, f"{map_name}.yaml")

    Image.fromarray(pixels, mode="L").save(pgm_path)

    meta = {
        "image": f"{map_name}.pgm",
        "resolution": grid.resolution,
        "origin": [grid.origin[0], grid.origin[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w") as f:
        yaml.safe_dump(meta, f, default_flow_style=False)

    return pgm_path, yaml_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--bin", default=os.path.join(
            _PACKAGE_DIR, "..", "amiga_ros2_behavior_tree", "examples",
            "mv_10_60_sample.bin"),
        help="Mission fixture to extract the orchard JSON frame from.")
    ap.add_argument("--out", default=os.path.join(_PACKAGE_DIR, "maps"))
    ap.add_argument("--map-name", default="orchard_map")
    ap.add_argument(
        "--datum-lat", type=float, default=None,
        help="Local-frame origin latitude; defaults to the first tree's own.")
    ap.add_argument("--datum-lon", type=float, default=None)
    ap.add_argument("--tree-radius", type=float, default=0.5,
                     help="Canopy radius, metres -- same default as this "
                     "package's own tree_obstacle_radius param.")
    ap.add_argument("--resolution", type=float, default=0.1,
                     help="Metres per cell. Finer than the live sim's own "
                     "default (0.25) since this is a one-time export, not "
                     "rebuilt per query.")
    ap.add_argument("--margin", type=float, default=5.0)
    args = ap.parse_args()

    trees = load_trees_from_bin(args.bin)
    print(f"loaded {len(trees)} trees from {args.bin}")

    datum_lat = args.datum_lat if args.datum_lat is not None else trees[0]["lat"]
    datum_lon = args.datum_lon if args.datum_lon is not None else trees[0]["lon"]
    print(f"datum: lat={datum_lat}, lon={datum_lon}")

    obstacles = trees_to_obstacles(trees, datum_lat, datum_lon, args.tree_radius)
    grid = build_grid_map(obstacles, args.resolution, args.margin)
    print(f"grid: {grid.width}x{grid.height} cells at {grid.resolution} m/cell, "
          f"origin=({grid.origin[0]:.2f}, {grid.origin[1]:.2f})")

    pgm_path, yaml_path = write_map(grid, args.out, args.map_name)
    print(f"wrote {pgm_path}")
    print(f"wrote {yaml_path}")

    trees_json_path = os.path.join(args.out, f"{args.map_name}_trees.json")
    with open(trees_json_path, "w") as f:
        json.dump({
            "datum_lat": datum_lat,
            "datum_lon": datum_lon,
            "trees": trees,
        }, f, indent=2)
    print(f"wrote {trees_json_path} (raw tree list + datum, for provenance)")


if __name__ == "__main__":
    main()
