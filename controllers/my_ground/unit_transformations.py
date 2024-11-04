from consts import *


def floor_to_gps(floor):
    """Convert floor coordinates to GPS coordinates."""
    return (
        floor[0] / FLOOR_LENGTH * GPS_LENGTH - FLOOR_ADD,
        FLOOR_ADD - (floor[1] / FLOOR_LENGTH * GPS_LENGTH)
    )


def gps_to_floor(gps):
    """Convert GPS coordinates to floor coordinates."""
    return (
        int((gps[0] + FLOOR_ADD) / GPS_LENGTH * FLOOR_LENGTH),
        int((FLOOR_ADD - gps[1]) / GPS_LENGTH * FLOOR_LENGTH)
    )


