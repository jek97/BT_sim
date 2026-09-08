"""
gps.py

Lat/lon (degrees) -> local ENU metres, around a fixed datum -- the
forward direction of the SAME equirectangular approximation
amiga_ros2_behavior_tree/src/orchard_management.cpp already uses in
reverse (its add_meters_to_gps: metres -> lat/lon, for GetTreeInfo's own
x_offset/y_offset query parameters). Kept forward-only here because this
package needs the opposite direction: the orchard JSON's own lat/lon
trees converted INTO the local frame the robot's tf2 pose already lives
in, not the other way around.

The datum MUST be the same one this robot's navsat_transform_node uses
(amiga-ros2-nav/amiga_localization/config/base_ekf.yaml's own `datum:`),
or a tree's converted (x,y) will not line up with the robot's own map
frame -- see this package's README for how to keep them in sync.
"""
import math

EARTH_RADIUS_M = 6378137.0


def latlon_to_local(lat_deg, lon_deg, datum_lat_deg, datum_lon_deg):
    """(lat,lon) degrees -> (x,y) metres east/north of the datum."""
    datum_lat_rad = math.radians(datum_lat_deg)
    dlat_rad = math.radians(lat_deg - datum_lat_deg)
    dlon_rad = math.radians(lon_deg - datum_lon_deg)
    y = dlat_rad * EARTH_RADIUS_M
    x = dlon_rad * EARTH_RADIUS_M * math.cos(datum_lat_rad)
    return x, y
