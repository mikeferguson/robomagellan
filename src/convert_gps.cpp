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

#include <iomanip>
#include <iostream>
#include <sstream>
#include <robomagellan/convert_gps.hpp>

namespace robomagellan
{

ConvertGPS::ConvertGPS() : ctx_(nullptr), transform_(nullptr)
{
}

ConvertGPS::~ConvertGPS()
{
  proj_destroy(transform_);
  proj_context_destroy(ctx_);
}

bool ConvertGPS::setDatum(double lat, double lon, double alt)
{
  std::cout << "Setting coordinate frame at " << lat << ", " << lon << std::endl;
  ctx_ = proj_context_create();
  std::stringstream ss;
  ss << "proj=pipeline step proj=cart ellps=WGS84 step proj=topocentric";
  ss << " lat_0=" << std::setprecision(10) << lat;
  ss << " lon_0=" << std::setprecision(10) << lon;
  ss << " h_0=" << std::setprecision(10) << alt;
  transform_ = proj_create(ctx_, ss.str().c_str());
  if (transform_ == 0)
  {
    std::cerr << "Failed to create cartesian transformation object" << std::endl;
    return false;
  }
  return true;
}

bool ConvertGPS::LLAtoCart(double lat, double lon, double alt,
                           double * e, double * n, double * u)
{
  // Setup conversions, if needed
  if (!ctx_ && !setDatum(lat, lon, alt))
  {
    return false;
  }

  // Do conversion
  PJ_COORD lla = proj_coord(proj_torad(lon), proj_torad(lat), alt, 0);
  PJ_COORD cart = proj_trans(transform_, PJ_FWD, lla);

  // Export result
  *e = cart.enu.e;
  *n = cart.enu.n;
  *u = cart.enu.u;

  return true;
}

bool ConvertGPS::CartToLLA(double e, double n, double u,
                           double * lat, double * lon, double * alt)
{
  if (!ctx_)
  {
    // Need to be setup
    std::cerr << "Datum not set, cannot convert" << std::endl;
    return false;
  }

  // Do conversion
  PJ_COORD cart = proj_coord(e, n, u, 0);
  PJ_COORD lla = proj_trans(transform_, PJ_INV, cart);

  // Export result
  *lon = proj_todeg(lla.lpz.lam);
  *lat = proj_todeg(lla.lpz.phi);
  *alt = lla.lpz.z;

  return true;
}

bool ConvertGPS::ready()
{
  return (transform_ != nullptr);
}

}  // namespace robomagellan
