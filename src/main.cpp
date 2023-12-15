#include <iostream>
#include <robomagellan/gps.hpp>

int main(int argc, char** argv)
{
  double lat = 0;
  double lon = 0;
  double alt = 0.0;

  robomagellan::GPS * gps = new robomagellan::GPS();

  double e, n, u;
  gps->LLAtoCart(lat, lon, alt, &e, &n, &u);
  std::cout << "Easting: " << e << " Northing: " << n << " Up: " << u << std::endl;

  lat = 0;
  lon = 0;
  gps->CartToLLA(e, n, u, &lat, &lon, &alt);
  std::cout << "Lat: " << lat << " Lon: " << lon << " Alt: " << alt << std::endl;

  delete gps;
  return 0;
}
