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

#include <robomagellan/waypoint_transformer.hpp>
#include <robot_localization/FromLL.h>

WaypointTransformer::WaypointTransformer()
{
  client_ = nh_.serviceClient<robot_localization::FromLL>("fromLL", true /* persistent */);
}

bool WaypointTransformer::transform(Path& path)
{
  if (!client_)
  {
    ROS_ERROR("Unable to transform, service not connnected");
    return false;
  }

  robot_localization::FromLL::Request req;
  robot_localization::FromLL::Response res;

  // Transform into map frame
  for (auto waypoint : path.waypoints)
  {
    req.ll_point.latitude = waypoint->latitude;
    req.ll_point.longitude = waypoint->longitude;
    req.ll_point.altitude = 0.0;

    client_.call(req, res);

    waypoint->x = res.map_point.x;
    waypoint->y = res.map_point.y;
  }

  // Set the headings of each point
  setHeadings(path);

  return true;
}
