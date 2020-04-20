/*
Copyright (c) 2020 Michael E. Ferguson. All right reserved.

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
*/

#ifndef ROBOMAGELLAN_PATH_HPP
#define ROBOMAGELLAN_PATH_HPP

#include <cmath>
#include <vector>
#include <robomagellan/waypoint.hpp>

typedef std::vector<Waypoint> Path;

inline double getHeading(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	return atan2(dy, dx);
}

inline bool setHeadings(Path& path)
{
	if (path.size() < 2)
	{
		// Not enough points
		return false;
	}

  // Set heading of each waypoint
  double heading;
  for (size_t i = 0; i < path.size() - 1; ++i)
  {
    heading = getHeading(path[i].x,
                         path[i].y,
                         path[i+1].x,
                         path[i+1].y);
    path[i].heading = heading;
  }

  // Set last point heading
  path.back().heading = heading;

  return true;
}

#endif  // ROBOMAGELLAN_PATH_HPP
