// Davi's bordeerjar script 
// Note: this is used in the bordejar function in the sailboat main code
// JL, MS 5/13/18


struct location angleToLocation(float lat, float lon) {
  location temp;
  temp.latitude = lat;
  temp.longitude = lon;
  return temp;
}

struct location projecao2d_mod(float lat, float lon, float a, float b, float a_p, float b_p)
{
  if (a == 0) {
    a = 0.00000001;
  }
  if (b == 0) {
    b = 0.00000001;
  }
  float a_aux = -1 / a;
  float b_aux = -a_aux * lat + lon;

  float latProj = (b_aux - b_p) / (a_p - a_aux);
  float lonProj = a_p * latProj + b_p;

  return angleToLocation(latProj, lonProj);
}

struct location projecao2d(float lat, float lon, float a, float b)
{
  if (a == 0) {
    a = 0.00000001;
  }
  if (b == 0) {
    b = 0.00000001;
  }
  float a_aux = -1 / a;
  float b_aux = -a_aux * lat + lon;

  float latProj = (b_aux - b) / (a - a_aux);
  float lonProj = a * latProj + b;

  return angleToLocation(latProj, lonProj);
}