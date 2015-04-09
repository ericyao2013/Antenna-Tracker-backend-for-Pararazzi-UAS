/* Simple library to calculate joint angles of tracker.. 
calc_joint_ang() fuction uses Haversine formula to calculate required joint angles.

Please check;
http://rosettacode.org/wiki/Haversine_formula
for implimentations. 

*/


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double RadOfDeg(double deg) {
  return (deg * M_PI / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double DegOfRad(double rad) {
  return (rad * 180 / M_PI);
}


#define R 6371
#define TO_RAD (3.1415926536 / 180)

void calc_joint_ang(double lat1, double lon1, double lat2, double lon2, float AcAlt, float TrAlt,  float *fPan, float *fTilt )
{
  double dx, dy, dz, dlon, PanDeg, Distance;
  dlon = lon2 - lon1; 
  
  PanDeg = DegOfRad( atan2( (-1)*(sin(dlon) * cos(lat2)), ( (cos(lat1) * sin(lat2) - sin(lat1)*cos(lat2)*cos(dlon)  ) ) ) );
  printf("PANDEG ==== %f\n", PanDeg );

  lon1 -= lon2;
  lon1 *= TO_RAD, lat1 *= TO_RAD, lat2 *= TO_RAD;
 
  
  
  dz = sin(lat1) - sin(lat2);
  dx = cos(lon1) * cos(lat1) - cos(lat2);
  dy = sin(lon1) * cos(lat1);
  Distance = 1000 * asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
  printf("Distance ==== %f\n", Distance );

  *fTilt = DegOfRad( atan2( (AcAlt-TrAlt), Distance) );
  *fPan = PanDeg;

}


