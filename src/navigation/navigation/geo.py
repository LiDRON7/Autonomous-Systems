"""Small coordinate helpers used by mission services."""

import math

EARTH_RADIUS_M = 6_378_137.0


def gps_to_ned(
    latitude: float,
    longitude: float,
    altitude: float,
    reference: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert WGS84 coordinates to a local NED approximation."""
    ref_lat, ref_lon, ref_alt = reference
    north = math.radians(latitude - ref_lat) * EARTH_RADIUS_M
    east = (
        math.radians(longitude - ref_lon)
        * EARTH_RADIUS_M
        * math.cos(math.radians(ref_lat))
    )
    return north, east, ref_alt - altitude
