/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOMAGELLAN__CONVERT_GPS_HPP_
#define ROBOMAGELLAN__CONVERT_GPS_HPP_

#include <proj.h>

namespace robomagellan
{
class ConvertGPS
{
public:
  ConvertGPS();
  virtual ~ConvertGPS();

  /**
   * @brief Setup the datum that will be the reference of the cartesian frame.
   * @param lat Latitude, in degrees.
   * @param lon Longitude, in degrees.
   * @param alt Altitude, in meters.
   */
  bool setDatum(double lat, double lon, double alt);

  /**
   * @brief Convert from Long-Lat-Altitude to cartesian coordinates.
   * @param lat Latitude, in degrees.
   * @param lon Longitude, in degrees.
   * @param alt Altitude, in meters.
   * @param e Returned easting coordinate in cartesian frame.
   * @param n Returned northing coordinate in cartesian frame.
   * @param u Returned up coordinate in cartesian frame.
   */
  bool LLAtoCart(double lat, double lon, double alt,
                 double * e, double * n, double * u);

  /**
   * @brief Convert from cartesian to Long-Lat-Altitude.
   * @param e Easting coordinate in cartesian frame.
   * @param n Northing coordinate in cartesian frame.
   * @param u Up coordinate in cartesian frame.
   * @param lat Latitude, in degrees.
   * @param lon Longitude, in degrees.
   * @param alt Altitude, in meters.
   */
  bool CartToLLA(double e, double n, double u,
                 double * lat, double * lon, double * alt);

  /**
   * @brief Is the converter ready? Currently that means datum is set.
   */
  bool ready();

private:
  PJ_CONTEXT * ctx_;
  PJ * transform_;
};

}  // namespace robomagellan

#endif  // ROBOMAGELLAN__CONVERT_GPS_HPP_
