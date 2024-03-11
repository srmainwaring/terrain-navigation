#!/usr/bin/env python3

"""
Test geoid_height
"""

import pygeodesy
from pygeodesy.ellipsoidalKarney import LatLon

# https://geographiclib.sourceforge.io/C++/doc/geoid.html#geoidinst
GEOID_FILE = "/usr/local/share/GeographicLib/geoids/egm96-5.pgm"
GEOID_INTERPOLATOR = pygeodesy.GeoidKarney(GEOID_FILE)


def geoid_height(lat, lon):
    """
    Get the geoid height at position: lat, lon.
    """
    position = LatLon(lat, lon)
    N = GEOID_INTERPOLATOR(position)
    return N


def main(args=None):
    """
    h: ellipsoid height
    H: orthometric height (AMSL)
    N: geoid height

    H = h - N

    or

    h = H + N
    """
    lat = 46.81320571899414
    lon = 9.847953796386719
    alt_H = 1562.0999755859375

    N = geoid_height(lat, lon)
    alt_h = alt_H + N

    print("lat:   {}".format(lat))
    print("lon:   {}".format(lon))
    print("alt_H: {}".format(alt_H))
    print("alt_h: {}".format(alt_h))
    print("N:     {}".format(N))


if __name__ == "__main__":
    main()
