// The code is for the GPS that Davi used 
// translated by JL 5/8/18

// update_position
void atualizar_posicao(struct location local){  
  unsigned long start = millis();
  unsigned long ms = 500;
  do 
  {
    while(Serial3.available())
    {
      int c = Serial3.read();
      if(gps.encode(c))
      {
        // gps.f_get_position(&flatitude, &flongitude, &age)
        //Get the position, where latitude and longitude are variables of float type, and the actual values are returned. Float types are easier to use, but result in larger and slower code. The age variable must be unsigned long type.
        gps.f_get_position(&local.latitude, &local.longitude);
        // direction
        direcao = gps.f_course();
        // velocity 
        velocidade = gps.f_speed_kmph();
      }
    }
  } while (millis() - start < ms);
  //distanciaAoDestino = (int)TinyGPS::distance_between(latitudeBarco, longitudeBarco, latitudeDestino, longitudeDestino);  
}

//  calculate_distance between two points 
int calcular_distancia(struct location p1, struct location p2){
  // distance_between - built in function in GPS library to find the distance b/w two points (struct with a lat and long)
  return (int)TinyGPS::distance_between(p1.latitude, p1.longitude, p2.latitude, p2.longitude);
}

//    calculate orientation
float calcular_orientacao(struct location p1, struct location p2){
  // Davi uses a built in function in the TinyGPS library to find the direction that you must travel (or orientation)
  float sp_aux = TinyGPS::course_to(p1.latitude, p1.longitude, p2.latitude, p2.longitude);
  /*Serial.print("p1->lat: ");
  Serial.print(p1.latitude, 6);
  Serial.print(" ");
  Serial.print("p1->lon: ");
  Serial.print(p1.longitude, 6);
  Serial.print(" ");
  Serial.print("p2->lat: ");
  Serial.print(p2.latitude, 6);
  Serial.print(" ");
  Serial.print("p2->lon: ");
  Serial.print(p2.longitude, 6);
  Serial.print(" ");
  Serial.print("Course to -> ");
  Serial.println(sp_aux);*/
  //float sp_aux = TinyGPS::course_to(-5.842960, -35.197244, -5.842988, -35.196384);
  
  // setting limit from [-180, 180] for the boat orientation angle 
  // e.g. if sp_aux is 181, then it is -179 
  if(sp_aux > 180){
    sp_aux = sp_aux-360;
  }
  // e.g. if sp_aux is -181, then it is 179
  if(sp_aux < -180){
    sp_aux = sp_aux+360;
  }
  return sp_aux;
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial3.available())
      gps.encode(Serial3.read());
  } while (millis() - start < ms);
}