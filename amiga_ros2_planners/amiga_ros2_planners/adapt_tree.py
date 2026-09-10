#!/usr/bin/env python3
"""
adapt_tree.py

Mechanically adapts a problog_project problems/<name>/behavior_tree.xml
root element to this repo's own convention, so it validates against
amiga_btcpp_planners.xsd and parses in bt_runner -- see this package's
README's own "Running a problog_project BT in this simulation" section
for the FULL checklist; this module only handles the one purely
mechanical step (root-element shape), not the semantic ones (BT.cpp leaf
registration, goal-point/obstacle-id remapping, etc. -- those ARE now
handled automatically elsewhere, see plan_service_node.py's
ProblogFrameTransform and OrchardObstacleStore.get_obstacle, but not
here).

What it does:
  - Adds a <Mission> element (first child of <root>) if one isn't
    already present -- required by amiga_btcpp_planners.xsd, absent from
    every problog_project tree (see --mission-text).
  - Adds/overwrites the root's own schema_location attribute.
  - Leaves main_tree_to_execute alone -- amiga_btcpp_planners.xsd now
    accepts it (see that file's own comment on the root element).
  - Leaves every other element/attribute byte-for-byte untouched.

A plain function (adapt/adapt_and_write), not just a CLI: this is what
lets run_problog_problem.launch.py import and call it directly at
launch-generation time, with no separate process/manual step needed --
see that launch file's own header.

CLI usage (ros2 run, or directly from a source checkout):
    ros2 run amiga_ros2_planners adapt_tree -- \
        --in .../problog_project/problems/problem0/behavior_tree.xml \
        --out /tmp/problem0_adapted.xml \
        --mission-text "plan a path with A* to the mission goal, then walk it"
"""
import argparse
import xml.etree.ElementTree as ET


def adapt(tree_path, mission_text, schema_location):
    """Returns an xml.etree.ElementTree.ElementTree -- call .write(...)
    on it, or use adapt_and_write below to do that in one step."""
    tree = ET.parse(tree_path)
    root = tree.getroot()
    if root.tag != "root":
        raise ValueError(f"{tree_path}: expected a <root> element, got <{root.tag}>")

    root.set("schema_location", schema_location)

    has_mission = any(child.tag == "Mission" for child in root)
    if not has_mission:
        mission = ET.Element("Mission")
        mission.text = mission_text
        root.insert(0, mission)

    return tree


def adapt_and_write(tree_path, out_path, mission_text, schema_location):
    """adapt() + write to `out_path` in one call -- what
    run_problog_problem.launch.py actually calls."""
    adapted = adapt(tree_path, mission_text, schema_location)
    adapted.write(out_path, encoding="unicode", xml_declaration=True)
    return out_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--in", dest="in_path", required=True)
    ap.add_argument("--out", dest="out_path", required=True)
    ap.add_argument(
        "--mission-text", default="problog_project mission (see behavior_tree.xml)",
        help="Text for the added <Mission> element, if the tree doesn't "
        "already have one.")
    ap.add_argument(
        "--schema-location", default="schemas/amiga_btcpp_planners.xsd",
        help="Value for the root element's own schema_location attribute.")
    args = ap.parse_args()

    adapt_and_write(args.in_path, args.out_path, args.mission_text, args.schema_location)
    print(f"wrote {args.out_path}")


if __name__ == "__main__":
    main()
