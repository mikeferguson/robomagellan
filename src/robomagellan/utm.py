"""
Copyright (c) 2014 Michael E. Ferguson. All right reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

from math import sqrt
from geodesy.utm import UTMPoint, fromLatLong

## @brief Get the metric distance between two [lat, lon, alt] arrays.
def metric_dist(a, b):
    pA = fromLatLong(a[0], a[1], a[2]).toPoint()
    pB = fromLatLong(b[0], b[1], b[2]).toPoint()

    dx2 = (pA.x - pB.x) * (pA.x - pB.x)
    dy2 = (pA.y - pB.y) * (pA.y - pB.y)
    dz2 = (pA.z - pB.z) * (pA.z - pB.z)

    return sqrt(dx2 + dy2 + dz2)

## @brief Get the metric distance between two nmea_msgs/Fix messages.
def metric_dist_from_msg(fix_a, fix_b):
    return metric_dist([fix_a.latitude, fix_a.longitude, fix_a.altitude],
                       [fix_b.latitude, fix_b.longitude, fix_b.altitude])

