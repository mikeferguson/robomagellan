/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved.
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
