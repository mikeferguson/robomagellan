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

import json
from .utm import metric_dist

## @brief Class representing a path of GPS waypoints, to be exported as geojson
class Path:

    ## @brief Constructor for a new path
    ## @param dist Optional minimum distance between points in path.
    def __init__(self, dist=-1):
        self.data = dict()
        self.data["type"] = "Feature"
        self.data["properties"] = dict()
        self.data["geometry"] = dict()
        self.data["geometry"]["type"] = "LineString"
        self.data["geometry"]["coordinates"] = list()
        self.messages = 0
        self.last_fix = [0, 0, 0]
        self.dist = dist

    ## @brief Add a GPS point to the path
    def addPoint(self, lat, lon, alt):
        # test if enough distance has been traveled
        if metric_dist(self.last_fix, [lat, lon, alt]) > self.dist:
            self._addPoint(lat, lon, alt)

    ## @brief Save path to a geojson file
    def save(self, filename):
        print("Exporting geojson containing " + str(self.messages) + " points")
        with open(filename, "w") as f:
            f.write(self.__str__())

    def __str__(self):
        return json.dumps(self.data, indent=4)

    def _addPoint(self, lat, lon, alt):
        self.last_fix = [lat, lon, alt]
        self.data["geometry"]["coordinates"].append( [lat, lon] )
        self.messages += 1

